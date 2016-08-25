/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the LABUST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Dula Nad
 *  Created: 05.03.2015.
 *********************************************************************/
#ifndef USBL_COMMS_ASCII6BIT_H
#define USBL_COMMS_ASCII6BIT_H

#include <string>
#include <vector>
#include <cstdint>

namespace labust
{
  namespace comms
  {
    ///\todo Add stream operators.
    struct Ascii6Bit
    {
      enum {digit_diff=48, alpha_diff=55};
      enum {alpha_limit_int=36, digit_limit_int=10};
      enum {char_size = 6};
      enum {point=36,
        comma,
        question_mark,
        exclamation_mark,
        space,
        newline = 62,
        zero_char};
      /**
       * Convert char from ascii to internal encoding.
       */
      static char to6Bit(char c)
      {
        if (isalpha(c)) return (toupper(c)-alpha_diff);
        if (isdigit(c)) return (c-digit_diff);
        if (c == '.') return (point);
        if (c == ',') return (comma);
        if (c == '?') return (question_mark);
        if (c == '!') return (exclamation_mark);
        if (c == ' ') return (space);
        if (c == '\n') return (newline);
        return zero_char;
      }
      /**
       * Conver char from internal endoascii to e
       */
      static char from6Bit(char c)
      {
        if (c<digit_limit_int) return (c+digit_diff);
        if (c<alpha_limit_int) return (c+alpha_diff);
        if (c == point) return '.';
        if (c == comma) return ',';
        if (c == question_mark) return '?';
        if (c == exclamation_mark) return '!';
        if (c == space) return ' ';
        if (c == newline) return '\n';
        return 0;
      }

      static std::string charToString(const std::vector<uint8_t>& msg)
      {
        std::string retVal(msg.size(),'\0');
        for (int i=0; i<msg.size(); ++i) retVal[i] = from6Bit(msg[i]);
        return retVal;
      }
    };
  };
}
/* USBL_COMMS_ASCII6BIT_H */
#endif



