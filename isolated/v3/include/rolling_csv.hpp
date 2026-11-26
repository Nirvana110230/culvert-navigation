#pragma once

#include <cerrno>
#include <cstdio>
#include <fstream>
#include <string>
#include <system_error>

// Rotate only between complete records. Each volume has its own CSV header.
class RollingCsv {
 public:
  RollingCsv(const std::string& path, const std::string& header)
      : path_(path), header_(header) {
    stream_.exceptions(std::ios::badbit | std::ios::failbit);
    open();
  }

  std::ostream& row() {
    if (stream_.tellp() >= std::streamoff(1024 * 1024)) {
      stream_.close();
      if (std::rename(path_.c_str(), (path_ + ".1").c_str()) != 0)
        throw std::system_error(errno, std::generic_category(), "rotate " + path_);
      open();
    }
    return stream_;
  }

  void flush() { stream_.flush(); }
  void close() { stream_.close(); }

 private:
  void open() {
    stream_.open(path_, std::ios::out | std::ios::trunc);
    stream_ << header_ << '\n';
  }
  std::string path_;
  std::string header_;
  std::ofstream stream_;
};
