# build_script.py
import os
import json

def get_all_files(directory):
    files = []
    valid_extensions = ('.xml', '.obj', '.stl', '.png')
    for root, _, filenames in os.walk(directory):
        for filename in filenames:
            if filename.lower().endswith(valid_extensions):
                path = os.path.join(root, filename)
                # Convert to forward slashes and remove the base directory
                rel_path = path.replace(directory, '').replace('\\', '/').lstrip('/')
                files.append(rel_path)
    return files

# Generate files.json
scene_dir = './examples/scenes'
files = get_all_files(scene_dir)
with open(os.path.join(scene_dir, 'files.json'), 'w') as f:
    json.dump(files, f, indent=2)