import os
import sys

def append_unique_lines_to_bashrc(source_file):
    bashrc_path = os.path.expanduser("~/.bashrc")

    # Read current .bashrc contents
    with open(bashrc_path, "r") as bashrc:
        bashrc_lines = bashrc.readlines()

    # Read lines to add
    with open(source_file, "r") as source:
        lines_to_add = source.readlines()

    # Remove any duplicates and strip whitespace
    new_lines = [line.rstrip() for line in lines_to_add if line.strip() and line not in bashrc_lines]

    if new_lines:
        with open(bashrc_path, "a") as bashrc:
            bashrc.write("\n# --- Appended lines start ---\n")
            for line in new_lines:
                bashrc.write(f"{line}\n")
            bashrc.write("# --- Appended lines end ---\n")
        print(f"✅ Added {len(new_lines)} new line(s) to ~/.bashrc")
    else:
        print("ℹ️ No new lines were added. All lines already exist.")

if __name__ == "__main__":
    append_unique_lines_to_bashrc(os.path.expanduser("/ws/config/append_to_bashrc.txt"))


    # If there are arguments passed to this container (i.e., CMD), run them
    if len(sys.argv) > 1:
        os.execvp(sys.argv[1], sys.argv[1:])
