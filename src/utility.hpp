#pragma once 

namespace io {

bool iseol(char c) {
  return (c == '\n') || (c == '\r');
}
bool iscomma(char c) {
  return c == ',';
}

}