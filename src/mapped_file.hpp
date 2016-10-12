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

template <class T>
const T* as(const char* first) {
	return *reinterpret_cast<const T*>(first);
}
template <class T>
const T* as(const char* first, const char* last) {
	if (std::distance(first, last) < sizeof(T)) {
		return nullptr;
	}
	return as<T>(first);
}

template <class T>
T* as(char* first) {
	return *reinterpret_cast<T*>(first);
}
template <class T>
T* as(char* first, char* last) {
	if (std::distance(first, last) < sizeof(T)) {
		return nullptr;
	}
	return as<T>(first);
}