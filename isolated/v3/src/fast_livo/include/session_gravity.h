#pragma once

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace session_gravity {

class Error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

inline std::string required(const char* key) {
  const char* value = std::getenv(key);
  if (!value || !*value) throw Error(std::string("Missing session gravity setting: ") + key);
  return value;
}

class Recorder {
public:
  static Recorder& instance() {
    static Recorder recorder;
    return recorder;
  }

  void initialized(double gx, double gy, double gz) {
    if (!enabled_) return;
    if (initialized_) closeForReset();
    if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz))
      throw Error("Estimator initialization produced nonfinite gravity");
    std::ostringstream body;
    body.imbue(std::locale::classic());
    body << std::setprecision(std::numeric_limits<double>::max_digits10)
         << "\"event\":\"initialized\",\"source\":\"ImuProcess::Process2/stat.gravity\","
         << "\"gravity_camera_init_m_s2\":[" << gx << ',' << gy << ',' << gz << ']';
    append(body.str(), true);
    initialized_ = true;
  }

  void closeForReset() {
    if (!initialized_) return;  // The ordinary first IMU initialization calls Reset().
    append("\"event\":\"segment_closed\",\"reason\":\"reinitialization_requested\"", false);
    throw Error("Gravity segment 1 closed before IMU reset. Start a new mapping session for segment 2.");
  }

private:
  bool enabled_ = false;
  bool initialized_ = false;
  std::string identity_;
  std::filesystem::path file_;

  Recorder() {
    if (!std::getenv("FAST_LIVO2_GRAVITY_DIR") && !std::getenv("FAST_LIVO2_SESSION_ID") &&
        !std::getenv("FAST_LIVO2_CONFIG_SHA256") && !std::getenv("FAST_LIVO2_ALGORITHM_SHA256"))
      return;  // Standalone estimator callers without a mapping session.
    const std::filesystem::path dir = required("FAST_LIVO2_GRAVITY_DIR");
    const std::string id = required("FAST_LIVO2_SESSION_ID");
    const std::string config = required("FAST_LIVO2_CONFIG_SHA256");
    const std::string algorithm = required("FAST_LIVO2_ALGORITHM_SHA256");
    auto digest = [](const std::string& s) {
      return s.size() == 64 && s.find_first_not_of("0123456789abcdef") == std::string::npos;
    };
    std::error_code error;
    if (!dir.is_absolute() || !std::filesystem::equivalent(dir, ".", error) || error ||
        dir.filename() != id || id.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-.") != std::string::npos ||
        !digest(config) || !digest(algorithm)) throw Error("Invalid session gravity identity or working directory");
    const auto executable = std::filesystem::canonical("/proc/self/exe", error).string();
    if (error || executable.find_first_of("\r\n\t") != std::string::npos)
      throw Error("Cannot identify the running estimator executable");
    std::ostringstream fields;
    fields << "\"session_id\":" << std::quoted(id) << ",\"resolved_sha256\":" << std::quoted(config)
           << ",\"algorithm_sha256\":" << std::quoted(algorithm) << ",\"executable\":" << std::quoted(executable)
           << ",\"segment\":1,\"pid\":" << getpid();
    identity_ = fields.str();
    file_ = dir / "gravity_events.jsonl";
    enabled_ = true;
  }

  void append(const std::string& body, bool first) const {
    const int flags = O_WRONLY | O_NOFOLLOW | O_CLOEXEC | (first ? O_CREAT | O_EXCL : O_APPEND);
    const int fd = ::open(file_.c_str(), flags, 0644);
    if (fd < 0) throw Error("Cannot open session gravity event: " + std::string(std::strerror(errno)));
    const std::string line = "{" + identity_ + "," + body + "}\n";
    std::size_t offset = 0;
    int failure = 0;
    while (offset < line.size()) {
      const auto wrote = ::write(fd, line.data() + offset, line.size() - offset);
      if (wrote < 0 && errno == EINTR) continue;
      if (wrote <= 0) { failure = wrote < 0 ? errno : EIO; break; }
      offset += static_cast<std::size_t>(wrote);
    }
    if (!failure && ::fsync(fd) != 0) failure = errno;
    if (::close(fd) != 0 && !failure) failure = errno;
    if (failure) throw Error("Cannot persist session gravity event: " + std::string(std::strerror(failure)));
  }
};

}  // namespace session_gravity
