new_http_archive(
  name = "boost",
  url = "https://sourceforge.net/projects/boost/files/boost/1.61.0/boost_1_61_0.tar.bz2/download",
  build_file = "lib/BUILD.boost",
  type = "tar.bz2",
  strip_prefix = "boost_1_61_0/",
  sha256 = "a547bd06c2fd9a71ba1d169d9cf0339da7ebf4753849a8f7d6fdb8feee99b640",
)

new_git_repository(
  name = "geometry",
  remote = "https://github.com/boostorg/geometry.git",
  build_file = "lib/BUILD.geometry",
  commit = "28c9d63b2ddffd3ad75eb1b1967d967c7794d851"
)
