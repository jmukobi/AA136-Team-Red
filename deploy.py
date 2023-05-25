"""
Python script to easily copy the code from local github repo directory to the Feather.
This makes version control easier by limiting barries to editing code in the local repo clone instead of on the Feather.
"""

import shutil
import os

# Set the source folder and destination folder path
#source_folder = 'src/'
source_folder = 'Feather M4/'

#Ensure this path matches the drive name of the Feather
destination_folder = "/media/jacob/CIRCUITPY"


for root, dirs, files in os.walk(destination_folder):
    for file in files:
        file_path = os.path.join(root, file)
        os.remove(file_path)
        print(f"Deleted: {file_path}")

for root, dirs, files in os.walk(source_folder):
    for file in files:
        file_path = os.path.join(root, file)
        relative_path = os.path.relpath(file_path, source_folder)
        destination_path = os.path.join(destination_folder, relative_path)
        os.makedirs(os.path.dirname(destination_path), exist_ok=True)
        shutil.copy2(file_path, destination_path)
        print(f"Copied: {file_path} to {destination_path}")

# Use the shutil.copytree() function to copy the entire directory tree from the source to the destination
#shutil.copytree(source_folder, destination_folder)

print('Folder contents successfully copied to the mounted drive!')
