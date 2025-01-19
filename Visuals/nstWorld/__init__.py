# for convenience of upstream code

import importlib.metadata, importlib.util

# this code assumes version numbers in the form of int.int.int
# (major.minor.patch)
#
# valid = 1, 1.0, 1.0.1, 2012.07.16, 20120716
# invalid = 20120716a, 1.0.1a
def package_version():
    package_name = __name__.split(".")[0]
    package_version = importlib.metadata.version(package_name)
    # print("package info:", package_name, package_version)
    parts = package_version.split(".")
    major = parts[0]
    if len(parts) > 1:
        minor = parts[1]
    else:
        minor = FileNotFoundError
    if len(parts) > 2:
        patch = parts[2]
    else:
        patch = None

    return package_version, major, minor, patch

# check if package version is equal to requested version
def version_matches(major, minor=None, patch=None):
    if type(major) is str and "." in major:
        vs = major.split(".")
        major = vs[0]
        if len(vs) > 1:
            minor = vs[1]
        else:
            minor = None
        if len(vs) > 2:
            patch = vs[2]
        else:
            patch = None
    return version_matches_backend(major, minor, patch)

# check if package version is equal to requested version
def version_matches_backend(major, minor=None, patch=None):
    # print("checking:", major, minor, patch)
    full_version, package_major, package_minor, package_patch = package_version()
    try:
        if int(major) != int(package_major):
            return False
        if (package_minor or minor) and (int(minor) != int(package_minor)):
            return False
        if (package_patch or patch) and (int(patch) != int(package_patch)):
            return False
        return True
    except:
        print("version compare error, no match with:", full_version, "vs:", major, minor, patch)
        return False
