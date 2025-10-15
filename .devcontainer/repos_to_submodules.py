"""Add git submodules from .repos files found in the src directory.
This script searches for .repos files in the src directory and adds the specified
git repositories as submodules to the current git repository.
"""

import glob
import os
import subprocess

import yaml

PREFIX = "src"


def add_git_submodule(repo_name, repo_url, repo_version):
    """Invoke `git submodule` command to add a repository as submodule.

    Args:
        repo_name (str): The name of the repository to add
        repo_url (str): The url of the repository to add
        repo_version (str): The version of the repository to add
    """
    subprocess.call(['git', 'submodule', 'add', '-b', repo_version, repo_url, repo_name])


def is_submodule(repo_name):
    """Invoke `git submodule` to find if the given repository is a submodule.

    Args:
        repo_name (str): The repository to investigate.

    Returns:
        bool: True if it's a submodule, False otherwise.
    """
    try:
        subprocess.check_output(['git', 'submodule', 'status', repo_name],
                                stderr=subprocess.DEVNULL)
        return True
    except subprocess.CalledProcessError:
        return False


def parse_repos_file(file_path):
    """Inspect the content of the given path for git repositories and and them as submodules.

    Args:
        file_path (str): The path to the folder that contains the repositories.
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        repos_data = yaml.safe_load(file)
        repositories = repos_data['repositories']

        for repo_name, repo_info in repositories.items():
            if 'type' in repo_info and repo_info['type'] == 'git':
                repo_url = repo_info['url']
                repo_version = repo_info['version']
                submodule_name = os.path.join(PREFIX, repo_name)

                if not is_submodule(submodule_name):
                    add_git_submodule(submodule_name, repo_url, repo_version)
                    print(f"Added {repo_name} as a submodule.")


# Find .repos files within the src directory
repos_files = glob.glob('src/**/*.repos', recursive=True)

# Process each .repos file
for repos_file in repos_files:
    parse_repos_file(repos_file)
