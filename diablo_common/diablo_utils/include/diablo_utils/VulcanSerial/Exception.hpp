// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DIABLO_UTILS__VULCANSERIAL__EXCEPTION_HPP_
#define DIABLO_UTILS__VULCANSERIAL__EXCEPTION_HPP_

// System includes

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace VulcanSerial
{

class Exception : public std::runtime_error
{
public:
  Exception(const char * file, int line, const std::string & arg) : std::runtime_error(arg)
  {
    msg_ = std::string(file) + ":" + std::to_string(line) + ": " + arg;
  }

  ~Exception() throw() {}

  const char * what() const throw() override { return msg_.c_str(); }

private:
  std::string msg_;
};

}  // namespace VulcanSerial

#define THROW_EXCEPT(arg) throw Exception(__FILE__, __LINE__, arg);

#endif  // DIABLO_UTILS__VULCANSERIAL__EXCEPTION_HPP_
