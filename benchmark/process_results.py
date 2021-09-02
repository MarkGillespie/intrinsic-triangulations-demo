#!/usr/bin/env python3

# Consume results from a dataset test, building a table for analysis and making plots

import os
import argparse

import numpy as np
import pandas as pd
import os
import sys
import signal
import random

def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def ensure_dir_exists(d):
    if not os.path.exists(d):
        os.makedirs(d)

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('dir', help="directory with result")
    parser.add_argument('--operation', type=str, default = "flipDelaunay", help="operation being tested (flipDelaunay or refineDelaunay)")
    parser.add_argument('--record_files', action='store_true')
    parser.add_argument('--max_files', type=int, default = -1, help="read in at most max_files many of the input files")
    parser.add_argument('--merged_files', default = "", help="csv of dataset")
    args = parser.parse_args()

    # Place results here
    out_dir = os.path.join(args.dir, "analysis")
    ensure_dir_exists(out_dir)

    # Parse in the individual CSVs
    frames = []
    bad_frames = []
    bad_files = []

    if args.merged_files:
        df = pd.read_csv(args.merged_files, header=0, sep=',',engine='python')
        bad_df = None
        print(df)
        print("Done loading df")
    else:
        data_files = os.listdir(args.dir)
        if (args.max_files > 0):
            data_files = os.listdir(args.dir)[:args.max_files]
        for i,f in enumerate(data_files):
            if not f.endswith(".tsv"):
                continue

            if i % 10 == 0:
                # print("loading " + str(f))
                wdth = 50
                progress = int(i / len(data_files) * wdth)
                bar = "[" + (progress  * "=") + ((wdth-progress)*" ") + "]"
                print(f"\rLoading Data {bar} ({int(progress / wdth * 100)}%)", end="", flush=True)
                # print("loading " + str(f))
            try :
                frame = pd.read_csv(os.path.join(args.dir,f), header=0, sep='\t',engine='python')

                # remove leading/trailing whitespace from column names
                frame = frame.rename(columns=lambda x:x.strip())

                # replace inf w/ np.inf, allowing for spaces around "inf"
                frame = frame.replace(to_replace=r"^\s*inf\s*$", value=np.inf, regex=True)

                # replace nan w/ np.nan, allowing for spaces around "nan"
                frame = frame.replace(to_replace=r"^\s*nan\s*$", value=np.nan, regex=True)

                # replace " false" w/ False, allowing for spaces around "false"
                frame = frame.replace(to_replace=r"^\s*false\s*$", value=False, regex=True)
            except:
                frame = pd.DataFrame()
            if frame.empty or "name" not in frame.columns:
                bad_files.append(f)
            elif pd.isnull(frame.at[0, 'inputVertices']):
                bad_frames.append(frame.head(1))
            else:
                frames.append(frame)
        print("\n")

        # Make a mega-frame
        if frames:
            df = pd.concat(frames, ignore_index=True)
        else:
            df = pd.DataFrame()

        # Save it
        df.to_csv(os.path.join(out_dir, "merged_results.csv"), index=False)

        bad_df = None
        if(len(bad_frames) > 0):
            bad_df = pd.concat(bad_frames, ignore_index=True)

        if bad_files:
            print(f"\n\n === bad files({len(bad_files)}) === ")
            print(bad_files)
        else:
            print("\n\n === all files parsed :) ")

        if bad_df is None:
            print("\n\n === no incomplete records :) ")
            filtered_bad_names = []
        else:
            print("\n\n === incomplete records === ")
            print(bad_df)

            bad_names = bad_df["name"].to_list()

            filtered_bad_names = [int(c) if len(c) else None for c in (''.join(takewhile(str.isdigit, str(x) or "")) for x in bad_names)]

        # print(bad_names)
        # print(filtered_bad_names)
        with open(f"bad_meshes.txt", "w") as f:
            f.write("\n".join(map(str, [x[:-4] for x in bad_files])))
            f.write("\n")
            f.write("\n".join(map(str, filtered_bad_names)))


        if args.record_files:
            with open("good_meshes.txt", "w") as f:
                f.write("\n".join(map(str,df["name"])))


    """
    # Filtering, etc
    df = df[df["minCornerAngle"] > 1e-6]

    # Compute some extra values
    df["flip_timeFrac"] = df["flip_flipTime"] / df["flip_totalTime"]

    # Print the table to the terminal
    """
    print(df)
    print(df.columns)

    n_bad_df = 0 if bad_df is None else bad_df.shape[0]

    n_total = len(df) + n_bad_df + len(bad_files)

    print(f"\tfailed to terminate successfully on {n_bad_df+len(bad_files)} meshes")

    if (args.operation == "flipDelaunay"):
        n_delaunay = df['outputIsDelaunay'].sum()
        print(f"Achieved Delaunay on {n_delaunay} of {n_total} meshes ({n_delaunay/n_total*100.}%)")
    elif args.operation == "refineDelaunay":
        angleBound = 25
        df['refinementSucceeded'] = df['outputMinValidAngleDeg'] >= angleBound
        n_good = df['refinementSucceeded'].sum()
        print(f"Achieved {angleBound} degree angle bound on {n_good} of {n_total} meshes ({n_good/n_total*100.}%)")

        df['extractionSucceeded'] = df['commonSubdivisionMeshingDuration'] > 0
        n_good_extraction = df['extractionSucceeded'].sum()
        print(f"extraced common refinement {n_good_extraction} of {n_total} meshes ({n_good_extraction/n_total*100.}%)")

        if args.record_files:
            unrefined_meshes = df[df['refinementSucceeded'] == False]['name']
            with open("unrefined_meshes", "w") as f:
                f.write("\n".join(map(str, unrefined_meshes)))
            unmeshed_meshes = df[df['extractionSucceeded'] == False]['name']
            with open("unmeshed_meshes", "w") as f:
                f.write("\n".join(map(str, unmeshed_meshes)))

if __name__ == "__main__":
    main()
