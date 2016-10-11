#pragma once

#include <boost/iostreams/device/mapped_file.hpp>

const char* begin(const boost::iostreams::mapped_file_source& file) {
	return file.data();
}

const char* end(const boost::iostreams::mapped_file_source& file) {
	return file.data() + file.size();
}

char* begin(const boost::iostreams::mapped_file_sink& file) {
  return file.data();
}

char* end(const boost::iostreams::mapped_file_sink& file) {
  return file.data() + file.size();
}

char* begin(const boost::iostreams::mapped_file& file) {
  return file.data();
}

char* end(const boost::iostreams::mapped_file& file) {
  return file.data() + file.size();
}