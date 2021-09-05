# intrinsic-triangulations-demo

Our core intrinsic triangulation data structures are implemented in [geometry-central](http://geometry-central.net). This is a simple appliction which loads a mesh, computes an intrinsic triangulation, and visualizes its edges.  Additionally, the code can be invoked from the command line to output data about the intrinsic triangulation in easily-parseable formats.  In this library, the resulting `IntrinsicTriangulation` class can be used with all geometry routines in geometry-central. 

### Building and running

On unix-like machines, use:
```
git clone --recurse-submodules https://github.com/nmwsharp/intrinsic-triangulations-demo.git
cd intrinsic-triangulations-demo
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
./bin/int_tri /path/to/your/mesh.obj
```

The codebase also builds on Visual Studio 2017 & 2019 (at least), by using CMake to generate a Visual Studio solution file.

Running the program open a UI window showing your mesh. The intrinsic triangulation is denoted by the colored edge tubes, whose thickness can be adjusted in the settings panel on the left.

The command window in the upper right can be used to flip the intrinsic triangulation to Delaunay, as well as perform Delaunay refinement. It also has options for outputting to file (see the command line documentation below).

### Command line interface

| flag | purpose | arguments |
| :------------- |:------------- | :-----|
| `--noGUI` | Do not show the GUI, just process options and exit | |
| `--backend` | Data structure to use (`signpost` or `integer`) | |
| `--flipDelaunay` | Flip edges to make the mesh intrinsic Delaunay | |
| `--refineDelaunay` | Refine and flip edges to make the mesh intrinsic Delaunay and satisfy angle/size bounds | |
| `--refineAngle` | Minimum angle threshold (in degrees). | the angle, default: `25.` |
| `--refineSizeCircum` | Maximum triangle size, set by specifying the circumradius. | the circumradius, default: `inf` |
| `--refineMaxInsertions` | Maximum number of insertions during refinement. Use 0 for no max, or negative values to scale by number of vertices. | the count, default: `-10` (= 10 * nVerts) |
| `--triangulateInput` | Triangulate the input mesh before running algorithms | |
| `--outputPrefix` |  Prefix to prepend to all output file paths | the prefix, default: `intrinsic_`|
| `--intrinsicFaces` | Write the face information for the intrinsic triangulation. These are two dense `Fx3` matrices, giving the indices of the vertices for each face, and the length of the edge from `i` to `(i+1)%3`'th adjacent vertex. Names: `faceInds.dmat`, `faceLengths.dmat` | |
| `--vertexPositions` | Write the vertex positions for the intrinsic triangulation. A dense `Vx3` matrix of 3D coordinates. Name: `vertexPositions.dmat` | |
| `--laplaceMat` | Write the Laplace-Beltrami matrix for the triangulation. A sparse `VxV` matrix, holding the _weak_ Laplace matrix (that is, does not include mass matrix). Name: `laplace.spmat` | |
| `--interpolateMat` | Write the matrix which expresses data on the intrinsic vertices as a linear combination of the input vertices. A sparse, `VxV` matrix where each row has up to 3 nonzero entries that sum to 1. The column indices will always be in the first `V_0` vertices, the original input vertices. Name: `interpolate.mat`| |
| `--functionTransferMat` | write the linear systems for L2-optimal function transfer between the input and intrinsic triangulations. name: `InputToIntrinsic_lhs.spmat`, `InputToIntrinsic_rhs.spmat`, etc. | |
| `--commonSubdivision` | write the common subdivision to an obj file. name: `common_subdivision.obj` | |
| `--logStats` | write performance statistics. name: `stats.tsv` | |

Notice that the vertices are indexed such that original input vertices appear first.

#### Output formats

Dense matrices are output as an ASCII file where each line is a row of the matrix, which entries separated by spaces. The first line is a comment prefixed by `#`, giving the number of rows and columns in the matrix. Such files can be automatically loaded in many environments (e.g numpy and matlab).

Sparse matrices are output as an ASCII file where each line one entry in the matrix, giving the row, column, and value. The row and column indices are **1-indexed** to make matlab happy. The first line is a comment prefixed by `#`, giving the number of rows and columns in the matrix. These files can be automatically loaded in matlab ([see here](https://www.mathworks.com/help/matlab/ref/spconvert.html)). Writing parsers in other environments should be straightforward.

#### Function transfer
If the `--functionTransferMat` flag is set, the executable will output four transfer matrices: `InputToIntrinsic_lhs.spmat`, `InputToIntrinsic_rhs.spmat`, `IntrinsicToInput_lhs.spmat`, and `IntrinsicToInput_rhs.spmat`. To transfer a function `f_intrinsic` from the intrinsic mesh to the input mesh, you simply solve the linear system
```
IntrinsicToInput_lhs * x = IntrinsicToInput_rhs * f_intrinsic
```
for a function `x` on the input mesh. Conversely, to transfer a function `f_input` from the input mesh to the intrinsic mesh, you can solve
```
InputToIntrinsic_lhs * x = InputToIntrinsic_rhs * f_input.
```

#### Statistics
If the `--logStats` flag is set, the executable will log performance statistics to `stats.tsv`. These include the mesh name, the number of vertices and minimum angle in the input mesh, the number of vertices and minimum angle in the computed intrinsic mesh, the number of vertices in the common subdivision, and how long it took to compute the common subdivision.
