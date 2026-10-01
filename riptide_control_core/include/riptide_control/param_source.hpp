#pragma once

#include <functional>
#include <string>
#include <vector>

namespace riptide
{

/// Framework-neutral parameter source for control laws: declare a parameter with
/// a default and read back its (possibly overridden) value in one call. The ROS
/// layer binds this to its node parameter interface, preserving parameter
/// declaration + override semantics without leaking the ROS framework into the
/// control-law core. Overloads cover the types the laws use (double, int, vector).
class ParamSource
{
public:
  virtual ~ParamSource() = default;

  virtual double declare(const std::string & name, double def) = 0;
  virtual int declare(const std::string & name, int def) = 0;
  virtual std::vector<double> declare(
    const std::string & name, const std::vector<double> & def) = 0;
};

/// Framework-neutral log sink for one-time configure() messages. The ROS layer
/// binds it to its logger; unit tests can pass a no-op.
using Logger = std::function<void(const std::string &)>;

}  // namespace riptide
