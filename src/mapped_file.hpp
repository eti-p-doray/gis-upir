#pragma once

#include <boost/iostreams/device/mapped_file.hpp>

class mapped_file {
 public:
  mapped_file(const char* name)
      : buffer_(name) {}
  const uint8_t* data() const { return reinterpret_cast<const uint8_t*>( buffer_.data() ); }
  size_t size() const { return buffer_.size(); }
  
  const uint8_t* begin() const { return reinterpret_cast<const uint8_t*>( buffer_.data() ); }
  const uint8_t* end() const { return reinterpret_cast<const uint8_t*>( buffer_.data() ) + size(); }
  const uint8_t* cbegin() const { return begin(); }
  const uint8_t* cend() const { return end(); }

 private:
  boost::iostreams::mapped_file_source buffer_;
};