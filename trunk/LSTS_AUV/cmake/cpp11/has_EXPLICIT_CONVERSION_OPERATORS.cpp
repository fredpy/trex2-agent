/**
 *  Copyright 2010-2012 Matus Chochlik. Distributed under the Boost
 *  Software License, Version 1.0. (See accompanying file
 *  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 */

class eco
{
public:
  explicit operator int (void) const
  {
    return 0;
  }
};

int main(int argc, const char** argv)
{
  eco x;
  return int(x);
}
