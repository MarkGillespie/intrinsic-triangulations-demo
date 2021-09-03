#include "geometrycentral/surface/integer_coordinates_intrinsic_triangulation.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/signpost_intrinsic_triangulation.h"
#include "geometrycentral/surface/surface_centers.h"
#include "geometrycentral/surface/transfer_functions.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "args/args.hxx"
#include "imgui.h"
#include "logger.h"

#include <sstream>

using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;
std::unique_ptr<IntrinsicTriangulation> intTri;
std::unique_ptr<CommonSubdivision> cs;

// Polyscope visualization handle, to quickly add data to the surface
bool withGUI = true;
polyscope::SurfaceMesh* psMesh;

// Parameters
std::string backend = "Integer Coordinates";
float refineToSize = -1;
float refineDegreeThresh = 25;
bool useRefineSizeThresh = false;
bool useInsertionsMax;
int insertionsMax = -2;

// Mesh stats
bool signpostIsDelaunay = true;
float signpostMinAngleDeg = 0.;

// Output options
std::string outputPrefix;

void updateTriagulationViz() {
  if (!withGUI) {
    return;
  }

  // Update stats
  signpostIsDelaunay = intTri->isDelaunay();
  signpostMinAngleDeg = intTri->minAngleDegrees();


  // Get the edge traces
  EdgeData<std::vector<SurfacePoint>> traces = intTri->traceAllIntrinsicEdgesAlongInput();

  // Convert to 3D positions
  std::vector<std::vector<Vector3>> traces3D(traces.size());
  size_t i = 0;
  for (Edge e : intTri->mesh.edges()) {
    for (SurfacePoint& p : traces[e]) {
      traces3D[i].push_back(p.interpolate(geometry->inputVertexPositions));
    }
    i++;
  }

  // Register with polyscope
  auto graphQ = polyscope::getSurfaceMesh()->addSurfaceGraphQuantity("intrinsic edges", traces3D);
  graphQ->setEnabled(true);
}

void resetTriangulation() {
  if (backend == "Integer Coordinates") {
    intTri.reset(new IntegerCoordinatesIntrinsicTriangulation(*mesh, *geometry));
  } else if (backend == "Signposts") {
    intTri.reset(new SignpostIntrinsicTriangulation(*mesh, *geometry));
  } else {
    throw std::runtime_error("unrecognized backed");
  }
  updateTriagulationViz();
}


void flipDelaunayTriangulation() {
  std::cout << "Flipping triangulation to Delaunay" << std::endl;
  intTri->flipToDelaunay();

  if (!intTri->isDelaunay()) {
    polyscope::warning("woah, failed to make mesh Delaunay with flips");
  }

  updateTriagulationViz();
}

void refineDelaunayTriangulation() {

  if (mesh->hasBoundary()) {
    // TODO: get rid of this message?
    if (withGUI) {
      polyscope::error("Support for refining meshes with boundary is experimental; proceed with caution!");
    } else {
      std::cout << "Support for refining meshes with boundary is experimental; proceed with caution!" << std::endl;
    }
  }

  // Manage optional parameters
  double sizeParam = useRefineSizeThresh ? refineToSize : std::numeric_limits<double>::infinity();
  size_t maxInsertions = useInsertionsMax ? insertionsMax : INVALID_IND;

  std::cout << "Refining triangulation to Delaunay with:   degreeThresh=" << refineDegreeThresh
            << " circumradiusThresh=" << refineToSize << " maxInsertions=" << maxInsertions << std::endl;

  intTri->delaunayRefine(refineDegreeThresh, sizeParam, maxInsertions);

  if (!intTri->isDelaunay()) {
    polyscope::warning(
        "Failed to make mesh Delaunay with flips & refinement. Bug Nick to finish porting implementation.");
  }

  updateTriagulationViz();
  std::cout << "\t...done" << std::endl;
}

void computeCommonSubdivision() {
  std::cout << "Computing common subdivision" << std::endl;
  intTri->getCommonSubdivision();
  if (withGUI) {
    // TODO: visualize
  }
  std::cout << "\t...done" << std::endl;
}

void testInterpolation() {
  if (!cs) computeCommonSubdivision();
  if (!cs->mesh) {
    std::cout << "Meshing common subdivision" << std::endl;
    cs->constructMesh();
    std::cout << "\t...done" << std::endl;
  }

  std::cout << "Comparing interpolated edge lengths " << std::endl;
  // Lengths from extrinsic vertex positions
  const VertexData<Vector3>& posCS = cs->interpolateAcrossA(geometry->vertexPositions);
  VertexPositionGeometry csGeo(*cs->mesh, posCS);
  csGeo.requireEdgeLengths();
  EdgeData<double> lengthsFromPosA = csGeo.edgeLengths;
  csGeo.unrequireEdgeLengths();

  // Lengths from extrinsic edge lengths
  geometry->requireEdgeLengths();
  const EdgeData<double>& lengthsA = geometry->edgeLengths;
  EdgeData<double> lengthsFromLenA = cs->interpolateEdgeLengthsA(lengthsA);

  // Lengths from intrinsic edge lengths
  EdgeData<double> lengthsFromLenB = cs->interpolateEdgeLengthsB(intTri->edgeLengths);

  std::cout << "   difference between interpolation from positionsA vs from lengthsA : "
            << (lengthsFromPosA.toVector() - lengthsFromLenA.toVector()).norm() << std::endl;
  std::cout << "   difference between interpolation from positionsA vs from lengthsA : "
            << (lengthsFromPosA.toVector() - lengthsFromLenB.toVector()).norm() << std::endl;
  std::cout << "   difference between interpolation from positionsA vs from lengthsA : "
            << (lengthsFromLenB.toVector() - lengthsFromLenA.toVector()).norm() << std::endl;

  std::cout << "Comparing mass matrices " << std::endl;

  SparseMatrix<double> massMatrixPosA = cs->vertexGalerkinMassMatrixFromPositionsA(geometry->vertexPositions);
  SparseMatrix<double> massMatrixLenA = cs->vertexGalerkinMassMatrixFromLengthsA(geometry->edgeLengths);
  SparseMatrix<double> massMatrixLenB = cs->vertexGalerkinMassMatrixFromLengthsB(intTri->edgeLengths);
  std::cout << "   difference between interpolation from positionsA vs from lengthsA : "
            << (massMatrixPosA - massMatrixLenA).norm() << std::endl;
  std::cout << "   difference between interpolation from positionsA vs from lengthsA : "
            << (massMatrixPosA - massMatrixLenB).norm() << std::endl;
  std::cout << "   difference between interpolation from positionsA vs from lengthsA : "
            << (massMatrixLenB - massMatrixLenA).norm() << std::endl;

  std::cout << "Comparing interpolation matrices " << std::endl;
  SparseMatrix<double> P_A = cs->interpolationMatrixA();
  Vector<double> vec_A = Vector<double>::Random(intTri->inputMesh.nVertices());
  VertexData<double> data_A(intTri->inputMesh, vec_A);
  VertexData<double> interp_fn_A = cs->interpolateAcrossA(data_A);
  Vector<double> vec_interp_A = P_A * vec_A;
  std::cout << "   difference between manual interpolation across A and interpolation matrix: "
            << (interp_fn_A.toVector() - vec_interp_A).norm() << std::endl;

  SparseMatrix<double> P_B = cs->interpolationMatrixB();
  Vector<double> vec_B = Vector<double>::Random(intTri->intrinsicMesh->nVertices());
  VertexData<double> data_B(*intTri->intrinsicMesh, vec_B);
  VertexData<double> interp_fn_B = cs->interpolateAcrossB(data_B);
  Vector<double> vec_interp_B = P_B * vec_B;
  std::cout << "   difference between manual interpolation across B and interpolation matrix: "
            << (interp_fn_B.toVector() - vec_interp_B).norm() << std::endl;

  geometry->unrequireEdgeLengths();
}

void testFunctionTransfer() {
  if (!cs) computeCommonSubdivision();
  if (!cs->mesh) {
    std::cout << "Meshing common subdivision" << std::endl;
    cs->constructMesh();
    std::cout << "\t...done" << std::endl;
  }

  std::cout << "Comparing attribute transfer" << std::endl;
  AttributeTransfer transfer(*cs, *geometry);
  VertexData<double> data_B(*intTri->intrinsicMesh, Vector<double>::Random(intTri->intrinsicMesh->nVertices()));
  VertexData<double> data_A_Pointwise = transfer.transferBtoA(data_B, TransferMethod::Pointwise);
  VertexData<double> data_A_L2 = transfer.transferBtoA(data_B, TransferMethod::L2);
  Vector<double> truth = transfer.P_B * data_B.toVector();
  Vector<double> pointwiseA = transfer.P_A * data_A_Pointwise.toVector();
  Vector<double> L2A = transfer.P_A * data_A_L2.toVector();

  auto pscs = polyscope::registerSurfaceMesh("common subdivision", cs->interpolateAcrossA(geometry->vertexPositions),
                                             cs->mesh->getFaceVertexList(), polyscopePermutations(*cs->mesh));
  pscs->addVertexScalarQuantity("truth", truth);
  pscs->addVertexScalarQuantity("pointwise", pointwiseA);
  pscs->addVertexScalarQuantity("L2", L2A);
  psMesh->addVertexScalarQuantity("pointwise", data_A_Pointwise);
  psMesh->addVertexScalarQuantity("l2", data_A_L2);

  double pointwiseErr = (pointwiseA - truth).dot(transfer.M_CS_Galerkin * (pointwiseA - truth));
  double L2Err = (L2A - truth).dot(transfer.M_CS_Galerkin * (L2A - truth));

  std::cout << "  pointwise err: " << pointwiseErr << std::endl;
  std::cout << "  L2 err: " << L2Err << std::endl;
}

template <typename T>
void saveMatrix(std::string filename, SparseMatrix<T>& matrix) {

  // WARNING: this follows matlab convention and thus is 1-indexed

  std::cout << "Writing sparse matrix to: " << filename << std::endl;

  std::ofstream outFile(filename);
  if (!outFile) {
    throw std::runtime_error("failed to open output file " + filename);
  }

  // Write a comment on the first line giving the dimensions
  outFile << "# sparse " << matrix.rows() << " " << matrix.cols() << std::endl;

  outFile << std::setprecision(16);

  for (int k = 0; k < matrix.outerSize(); ++k) {
    for (typename SparseMatrix<T>::InnerIterator it(matrix, k); it; ++it) {
      T val = it.value();
      size_t iRow = it.row();
      size_t iCol = it.col();

      outFile << (iRow + 1) << " " << (iCol + 1) << " " << val << std::endl;
    }
  }

  outFile.close();
}

template <typename T>
void saveMatrix(std::string filename, DenseMatrix<T>& matrix) {

  std::cout << "Writing dense matrix to: " << filename << std::endl;

  std::ofstream outFile(filename);
  if (!outFile) {
    throw std::runtime_error("failed to open output file " + filename);
  }

  // Write a comment on the first line giving the dimensions
  outFile << "# dense " << matrix.rows() << " " << matrix.cols() << std::endl;

  outFile << std::setprecision(16);

  for (size_t iRow = 0; iRow < (size_t)matrix.rows(); iRow++) {
    for (size_t iCol = 0; iCol < (size_t)matrix.cols(); iCol++) {
      T val = matrix(iRow, iCol);
      outFile << val;
      if (iCol + 1 != (size_t)matrix.cols()) {
        outFile << " ";
      }
    }
    outFile << std::endl;
  }

  outFile.close();
}

void outputIntrinsicFaces() {

  intTri->requireVertexIndices();

  // Assemble Fx3 adjacency matrices vertex indices and edge lengths
  size_t nV = intTri->mesh.nVertices();
  size_t nF = intTri->mesh.nFaces();

  DenseMatrix<double> faceLengths(nF, 3);
  DenseMatrix<size_t> faceInds(nF, 3);

  size_t iF = 0;
  for (Face f : intTri->mesh.faces()) {

    Halfedge he = f.halfedge();
    for (int v = 0; v < 3; v++) {

      Vertex vA = he.vertex();
      Vertex vB = he.twin().vertex();
      size_t indA = intTri->vertexIndices[vA];
      size_t indB = intTri->vertexIndices[vB];
      Edge e = he.edge();
      double l = intTri->edgeLengths[e];

      faceLengths(iF, v) = l;
      faceInds(iF, v) = indA;

      he = he.next();
    }

    iF++;
  }

  saveMatrix("faceInds.dmat", faceInds);
  saveMatrix("faceLengths.dmat", faceLengths);
}

void outputVertexPositions() {
  // TODO
  throw std::runtime_error("not implemented");

  /*
  intTri->requireVertexIndices();

  size_t nV = intTri->mesh.nVertices();
  DenseMatrix<double> vertexPositions(nV, 3);

  VertexData<Vector3> intrinsicPositions = intTri->sampleAtInput(geometry->inputVertexPositions);

  size_t iV = 0;
  for (Vertex v : intTri->mesh.vertices()) {
    Vector3 p = intrinsicPositions[v];
    vertexPositions(iV, 0) = p.x;
    vertexPositions(iV, 1) = p.y;
    vertexPositions(iV, 2) = p.z;
    iV++;
  }

  saveMatrix("vertexPositions.dmat", vertexPositions);
  */
}

void outputLaplaceMat() {
  intTri->requireCotanLaplacian();
  saveMatrix("laplace.spmat", intTri->cotanLaplacian);
}

void outputInterpolatMat() {
  intTri->requireVertexIndices();

  // Assemble Fx3 adjacency matrices vertex indices and edge lengths
  size_t nV = intTri->mesh.nVertices();

  SparseMatrix<double> interpMat(nV, nV);
  std::vector<Eigen::Triplet<double>> triplets;

  size_t iV = 0;
  for (Vertex v : intTri->mesh.vertices()) {
    SurfacePoint p = intTri->vertexLocations[v];
    p = p.inSomeFace();

    Face f = p.face;

    int j = 0;
    for (Vertex n : f.adjacentVertices()) {
      size_t jV = intTri->vertexIndices[n];
      double w = p.faceCoords[j];
      if (w > 0) {
        triplets.emplace_back(iV, jV, w);
      }
      j++;
    }
    iV++;
  }

  interpMat.setFromTriplets(triplets.begin(), triplets.end());
  interpMat.makeCompressed();

  saveMatrix("interpolate.spmat", interpMat);
}

void writeLog(const Logger& logger, std::string outputPrefix) {
  std::string logFile = outputPrefix + "stats.tsv";
  bool loggingSuccess = logger.writeLog(logFile);
  if (!loggingSuccess) {
    throw std::runtime_error("failed to write logs to " + logFile);
  }
}

void myCallback() {

  ImGui::PushItemWidth(100);

  const std::array<std::string, 2> items = {"Integer Coordinates", "Signposts"};
  if (ImGui::BeginCombo("##backendcombo", backend.c_str())) {
    for (size_t n = 0; n < items.size(); n++) {
      bool is_selected = (backend == items[n]);
      if (ImGui::Selectable(items[n].c_str(), is_selected)) {
        // set a new backend
        backend = items[n];
        resetTriangulation();
      }
      if (is_selected) ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }

  ImGui::TextUnformatted("Intrinsic triangulation:");
  ImGui::Text("  nVertices = %lu  nFaces = %lu", intTri->mesh.nVertices(), intTri->mesh.nFaces());
  if (signpostIsDelaunay) {
    ImGui::Text("  is Delaunay: yes  min angle = %.2f degrees", signpostMinAngleDeg);
  } else {
    ImGui::Text("  is Delaunay: no   min angle = %.2f degrees", signpostMinAngleDeg);
  }

  if (ImGui::Button("reset triangulation")) {
    resetTriangulation();
  }

  if (ImGui::TreeNode("Delaunay flipping")) {
    if (ImGui::Button("flip to Delaunay")) {
      flipDelaunayTriangulation();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Delaunay refinement")) {
    ImGui::InputFloat("degree threshold", &refineDegreeThresh);

    ImGui::Checkbox("refine large triangles", &useRefineSizeThresh);
    if (useRefineSizeThresh) {
      ImGui::InputFloat("size threshold (circumradius)", &refineToSize);
    }

    ImGui::Checkbox("limit number of insertions", &useInsertionsMax);
    if (useInsertionsMax) {
      ImGui::InputInt("num insertions", &insertionsMax);
    }

    if (ImGui::Button("Delaunay refine")) {
      refineDelaunayTriangulation();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Output")) {

    if (ImGui::Button("intrinsic faces")) outputIntrinsicFaces();
    if (ImGui::Button("vertex positions")) outputVertexPositions();
    if (ImGui::Button("Laplace matrix")) outputLaplaceMat();
    if (ImGui::Button("interpolate matrix")) outputInterpolatMat();

    ImGui::TreePop();
  }
  if (ImGui::Button("Test interpolation")) testInterpolation();
  if (ImGui::Button("Test transfer")) testFunctionTransfer();

  ImGui::PopItemWidth();
}

int main(int argc, char** argv) {

  // Configure the argument parser
  // clang-format off
  args::ArgumentParser parser("A demo of Navigating Intrinsic Triangulations");
  args::HelpFlag help(parser, "help", "Display this help message", {'h', "help"});
  args::Positional<std::string> inputFilename(parser, "mesh", "A .obj or .ply mesh file.");

  args::Group triangulation(parser, "triangulation");
  args::ValueFlag<std::string> backendFlag(triangulation, "backend", "Data structure to use (signpost or integer)", {"backend"});
  args::Flag flipDelaunay(triangulation, "flipDelaunay", "Flip edges to make the mesh intrinsic Delaunay", {"flipDelaunay"});
  args::Flag refineDelaunay(triangulation, "refineDelaunay", "Refine and flip edges to make the mesh intrinsic Delaunay and satisfy angle/size bounds", {"refineDelaunay"});
  args::ValueFlag<double> refineAngle(triangulation, "refineAngle", "Minimum angle threshold (in degrees). Default: 25.", {"refineAngle"}, 25.);
  args::ValueFlag<double> refineSizeCircum(triangulation, "refineSizeCircum", "Maximum triangle size, set by specifying the circumradius. Default: inf", {"refineSizeCircum"}, std::numeric_limits<double>::infinity());
  args::ValueFlag<int> refineMaxInsertions(triangulation, "refineMaxInsertions",
      "Maximum number of insertions during refinement. Use 0 for no max, or negative values to scale by number of vertices. Default: 10 * nVerts",
      {"refineMaxInsertions"}, -10);
  args::Flag triangulateInput(triangulation, "triangulateInput", "Triangulate non-triangular faces of input", {"triangulateInput"});

  args::Group output(parser, "ouput");
  args::Flag noGUI(output, "noGUI", "exit after processing and do not open the GUI", {"noGUI"});
  args::ValueFlag<std::string> outputPrefixArg(output, "outputPrefix", "Prefix to prepend to all output file paths. Default: intrinsic_", {"outputPrefix"}, "intrinsic_");
  args::Flag intrinsicFaces(output, "edgeLengths", "write the face information for the intrinsic triangulation. name: 'faceInds.dmat, faceLengths.dmat'", {"intrinsicFaces"});
  args::Flag vertexPositions(output, "vertexPositions", "write the vertex positions for the intrinsic triangulation. name: 'vertexPositions.dmat'", {"vertexPositions"});
  args::Flag laplaceMat(output, "laplaceMat", "write the Laplace-Beltrami matrix for the triangulation. name: 'laplace.spmat'", {"laplaceMat"});
  args::Flag interpolateMat(output, "interpolateMat", "write the matrix which expresses data on the intrinsic vertices as a linear combination of the input vertices. name: 'interpolate.mat'", {"interpolateMat"});
  args::Flag logStats(output, "logStats", "write performance statistics. name: 'stats.tsv'", {"logStats"});
  // clang-format on


  // Parse args
  try {
    parser.ParseCLI(argc, argv);
  } catch (const args::Help& h) {
    std::cout << parser;
    return 0;
  } catch (const args::ParseError& e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }

  // Make sure a mesh name was given
  if (!inputFilename) {
    std::cout << parser;
    return EXIT_FAILURE;
  }

  // Set options
  withGUI = !noGUI;

  refineDegreeThresh = args::get(refineAngle);
  refineToSize = args::get(refineSizeCircum);
  useRefineSizeThresh = refineToSize < std::numeric_limits<double>::infinity();
  insertionsMax = args::get(refineMaxInsertions);
  useInsertionsMax = insertionsMax != 0;
  outputPrefix = args::get(outputPrefixArg);

  if (logStats) {
    // output a temporary symbol so we can tell if the program
    // crashes before writing the real log

    std::string logFile = outputPrefix + "stats.tsv";
    std::ofstream out;

    // std::ios::trunc ensures that we overwrite old versions
    out.open(logFile, std::ios::trunc);
    if (out.is_open()) {
      out << ":'(" << std::endl;
      out.close();
    } else {
      std::cout << "Error: failed to write to " << logFile << std::endl;
    }
  }

  if (backendFlag) {
    if (args::get(backendFlag) == "signpost") {
      backend = "Signposts";
    } else if (args::get(backendFlag) == "integer") {
      backend = "Integer Coordinates";
    } else {
      std::cout << "Error: unrecognized backend '" << args::get(backendFlag) << "'. Please use 'signpost' or 'integer'"
                << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Load mesh
  std::tie(mesh, geometry) = readManifoldSurfaceMesh(args::get(inputFilename));

  if (triangulateInput && !mesh->isTriangular()) {
    std::cout << "triangulating faces..." << std::endl;
    for (Face f : mesh->faces()) mesh->triangulate(f);
  }

  // Sale max insertions by number of vertices if needed
  if (insertionsMax < 0) {
    insertionsMax *= -mesh->nVertices();
  }

  if (withGUI) {

    // Initialize polyscope
    polyscope::init();

    // Set the callback function
    polyscope::state::userCallback = myCallback;

    // Register the mesh with polyscope
    psMesh = polyscope::registerSurfaceMesh(polyscope::guessNiceNameFromPath(args::get(inputFilename)),
                                            geometry->inputVertexPositions, mesh->getFaceVertexList(),
                                            polyscopePermutations(*mesh));


    // Set vertex tangent spaces
    geometry->requireVertexTangentBasis();
    VertexData<Vector3> vBasisX(*mesh);
    for (Vertex v : mesh->vertices()) {
      vBasisX[v] = geometry->vertexTangentBasis[v][0];
    }
    polyscope::getSurfaceMesh()->setVertexTangentBasisX(vBasisX);

    // Set face tangent spaces
    geometry->requireFaceTangentBasis();
    FaceData<Vector3> fBasisX(*mesh);
    for (Face f : mesh->faces()) {
      fBasisX[f] = geometry->faceTangentBasis[f][0];
    }
    polyscope::getSurfaceMesh()->setFaceTangentBasisX(fBasisX);

    // Nice defaults
    psMesh->setEdgeWidth(1.0);
  }

  Logger logger;

  // Initialize triangulation
  resetTriangulation();

  if (logStats) {
    logger.log("name", polyscope::guessNiceNameFromPath(args::get(inputFilename)));
    logger.log("inputVertices", mesh->nVertices());
    logger.log("inputIsDelaunay", intTri->isDelaunay());
    logger.log("inputMinAngleDeg", intTri->minAngleDegrees());
    logger.log("inputMinValidAngleDeg", intTri->minAngleDegreesAtValidFaces(60));
  }

  // Perform any operations requested
  bool performedOperation = false;
  if (flipDelaunay) {
    std::clock_t start = std::clock();
    flipDelaunayTriangulation();
    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    if (logStats) logger.log("flippingDuration", duration);
    performedOperation = true;
  }

  if (refineDelaunay) {
    std::clock_t start = std::clock();
    refineDelaunayTriangulation();
    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    if (logStats) logger.log("refinementDuration", duration);
    performedOperation = true;
  }

  if (logStats) {
    logger.log("outputVertices", intTri->intrinsicMesh->nVertices());
    logger.log("outputIsDelaunay", intTri->isDelaunay());
    logger.log("outputMinAngleDeg", intTri->minAngleDegrees());
    logger.log("outputMinValidAngleDeg", intTri->minAngleDegreesAtValidFaces(60));

    if (performedOperation) {
      // log dummy value in case we time out
      logger.log("commonSubdivisionTracingDuration", -1);
      logger.log("commonSubdivisionMeshingDuration", -1);
      logger.log("commonSubdivisionVertices", -1);
    }

    if (logStats) writeLog(logger, outputPrefix);
  }

  if (performedOperation) {
    // compute common subdivision
    std::clock_t start = std::clock();
    computeCommonSubdivision();
    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    if (logStats) {
      logger.log("commonSubdivisionTracingDuration", duration);
      logger.log("commonSubdivisionVertices", cs->nVertices());
      writeLog(logger, outputPrefix);
    }

    // extract mesh of common subdivision
    start = std::clock();
    std::cout << "Constructing common subdivision mesh" << std::endl;
    cs->constructMesh();
    std::cout << "\t...done" << std::endl;
    duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    if (logStats) {
      logger.log("commonSubdivisionMeshingDuration", duration);
      writeLog(logger, outputPrefix);
    }
  }

  // Generate any outputs
  if (intrinsicFaces) outputIntrinsicFaces();
  if (vertexPositions) outputVertexPositions();
  if (laplaceMat) outputLaplaceMat();
  if (interpolateMat) outputInterpolatMat();

  if (logStats) writeLog(logger, outputPrefix);

  // Give control to the polyscope gui
  if (withGUI) {
    polyscope::show();
  }

  return EXIT_SUCCESS;
}
