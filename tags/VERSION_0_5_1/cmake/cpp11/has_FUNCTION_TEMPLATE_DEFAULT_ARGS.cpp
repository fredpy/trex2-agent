/**
 *  Copyright 2010-2012 Matus Chochlik. Distributed under the Boost
 *  Software License, Version 1.0. (See accompanying file
 *  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 */

struct ftda
{
  template <typename T = int>
  T func(T p = T())
  {
    return p;
  }
};

int main(int argc, const char** argv)
{
  ftda x;
  return x.func();
}
