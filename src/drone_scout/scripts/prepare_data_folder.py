#!/usr/bin/env python3

import os
import datetime
import argparse # For parsing command-line arguments

def prepare_data_folder(folder_path):
    """
    Manages a specified data folder.
    If the folder exists and is not empty, it renames it with a timestamp.
    Then, it ensures a new, empty folder exists at the original path.
    """
    print(f"Preparing data folder: '{folder_path}'")
    
    if os.path.exists(folder_path):
        print(f"Data folder '{folder_path}' already exists.")
        # Check if the folder is empty
        if not os.listdir(folder_path):
            print(f"Folder '{folder_path}' is empty. No renaming needed.")
        else:
            # Folder exists and is not empty, rename it
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            new_folder_name = f"{os.path.basename(folder_path)}_{timestamp}"
            parent_dir = os.path.dirname(folder_path)
            renamed_path = os.path.join(parent_dir, new_folder_name)

            try:
                os.rename(folder_path, renamed_path)
                print(f"Renamed existing data folder from '{folder_path}' to '{renamed_path}'.")
            except OSError as e:
                print(f"ERROR: Error renaming folder '{folder_path}': {e}")
                # Exit with a non-zero status code to indicate failure
                exit(1) 

    # Ensure the original folder path exists and is empty (either newly created or was already empty)
    try:
        os.makedirs(folder_path, exist_ok=True) # exist_ok=True prevents error if folder already exists
        print(f"Ensured empty data folder exists at '{folder_path}'.")
    except OSError as e:
        print(f"ERROR: Error creating data folder '{folder_path}': {e}")
        exit(1) # Exit with a non-zero status code to indicate failure

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Prepare a data folder by renaming it if not empty and ensuring an empty one exists.")
    parser.add_argument('folder_path', type=str, help='The path to the data folder to prepare.')
    args = parser.parse_args()

    prepare_data_folder(args.folder_path)

