/**
 *  Copyright 2010-2012 Matus Chochlik. Distributed under the Boost
 *  Software License, Version 1.0. (See accompanying file
 *  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 *  Checks for the constexpr keyword
 */

enum class Test : unsigned char {
  A = 'A',
  B = 'B',
  C = 'C'
};

int main(int argc, const char** argv)
{
  Test t = Test::A;
  return char(t) == 'A'?0:1;
}
