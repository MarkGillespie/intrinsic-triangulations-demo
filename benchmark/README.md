## Running a benchmark
`run_benchmark.py` runs the code on a dataset that you provide and records the algorithm's performance.
The script produces a `tsv` file recording some performance statistics for each mesh in the input. These files can then be analyed by `summarize_results.py`.

|flag | purpose |
| ------------- |-------------| 
|`--dataset_dir=/path/to/meshes`| Directory of mesh files to run on (required). |
|`--output_dir=/path/to/output/dir`| Directory to store results (required). |
|`--good_list=meshes_to_use.txt`| File of meshes which should be used. By default, all meshes are used. |
|`--bad_list=meshes_to_skip.txt`| File of meshes which should be excluded. |
|`--n_threads=1` | Number of threads to run on (default=1). |
|`--timeout=600` | Timeout in seconds (default=600). |
|`--operation` | Operation to perform (`flipDelaunay` or `refineDelaunay`) |
|`--max_meshes=1000` | Maximum number of meshes to process. By default, all meshes are used. |

## Summarizing benchmark results
Run `process_results.py your_output_dir` to summarize the results (saved to `your_output_dir/analysis`) along with a `csv` containing the statistics from all meshes.

|flag | purpose|
| ------------- |-------------| 
|`--operation` | Operation to use (`flipDelaunay` or `refineDelaunay`; required). |
|`--merged_files` | Name of a `csv` to read, instead of reading in individual mesh records. |
