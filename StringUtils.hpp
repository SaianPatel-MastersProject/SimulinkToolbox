// Useful string functions
// Copyright (c) 2010-2018 Kangaloosh Ltd t/a rFpro, All Rights Reserved.
//
// NOTICE:  All information contained herein is, and remains the property of rFpro. The intellectual and technical concepts contained
// herein are proprietary to rFpro and may be covered by U.S. and foreign patents, patents in process, and are protected by trade secret or copyright law.
// Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained from rFpro.
//
// The copyright notice above does not evidence any actual or intended publication or disclosure of this source code, which includes information that is confidential
// and/or proprietary, and is a trade secret, of rFpro.  ANY REPRODUCTION, DISTRIBUTION, PUBLIC PERFORMANCE, OR PUBLIC DISPLAY OF THIS SOURCE CODE
// WITHOUT THE EXPRESS WRITTEN CONSENT OF RFPRO IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL TREATIES.
// THE RECEIPT OR POSSESSION OF THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS TO REPRODUCE,
// DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT MAY DESCRIBE, IN WHOLE OR IN PART.

#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <cctype>
#include <cassert>

namespace StringUtils
{
  inline std::string ToLower(std::string s)
  {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
  }

  inline std::string ToUpper(std::string s)
  {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
  }

  inline bool IsCaseInsensitiveEqual(const std::string& s1, const std::string& s2)
  {
    return s1.size() == s2.size()
      && std::equal(s1.begin(), s1.end(), s2.begin(), [](const char &a, const char &b) {return toupper(a) == toupper(b); });
  }

  inline bool HasSuffix(const std::string &str, const std::string &suffix)
  {
    return str.size() >= suffix.size() &&
      std::equal(suffix.rbegin(), suffix.rend(), str.rbegin());
  }

  // Implementation of .NET Path.GetFileName
  inline std::string GetFileName(const std::string &filename)
  {
    const char* separators = ":\\/";
    const size_t start = filename.find_last_of(separators);
    if (start == std::string::npos)
    {
      return filename;
    }
    else
    {
      return filename.substr(start + 1);
    }
  }

  inline std::string ReplaceFileExtension(const std::string &filename, const std::string &newExtension)
  {
    auto extensionSeperator = filename.find_last_of('.');
    if (extensionSeperator != std::string::npos)
    {
      auto copy(filename);
      return copy.replace(extensionSeperator + 1, std::string::npos, newExtension);
    }
    else
    {
      return filename + '.' + newExtension;
    }
  }

  inline void ReplaceBackslash(std::string &path)
  {
    std::replace(path.begin(), path.end(), '\\', '/');
  }

  inline size_t GetNextPositiveInteger(const std::string &str, unsigned int *val, size_t pos = 0)
  {
    assert(0 != val);
    const char numerals[] = "1234567890";
    size_t s = str.find_first_of(numerals, pos);
    if (std::string::npos == s)
    {
      *val = 0;
      return s;
    }
    else
    {
      size_t e = str.find_first_not_of(numerals, s);
      if (std::string::npos == e)
      {
        e = str.length();
      }
      *val = atoi(str.substr(s, e).c_str());
      return e;
    }
  }

  inline size_t GetNextInteger(const std::string &str, int *val, size_t pos = 0)
  {
    assert(0 != val);
    const char numerals[] = "1234567890";
    size_t s = str.find_first_of(numerals, pos);
    if (std::string::npos == s)
    {
      *val = 0;
      return s;
    }
    else
    {
      size_t e = str.find_first_not_of(numerals, s);
      if (std::string::npos == e)
      {
        e = str.length();
      }
      *val = atoi(str.substr(s, e).c_str());
      if (s > pos &&
        str[s - 1] == '-')
      {
        *val = -*val;
      }
      return e;
    }
  }

  // Remove whitespace characters from either (or both) ends of the string [in-place]
  inline void StripWhitespaceFromBegining(std::string &s)
  {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {return !std::isspace(ch); }));
  }
  inline void StripWhitespaceFromEnd(std::string &s)
  {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) { return !std::isspace(ch); }).base(), s.end());
  }
  inline void StripWhitespace(std::string &s)
  {
    StripWhitespaceFromBegining(s);
    StripWhitespaceFromEnd(s);
  }

  inline void StripQuotes(std::string &s)
  {
    if (s.size() >= 2 && s.front() == '\"' && s.back() == '\"')
    {
      s = s.substr(1, s.size() - 2);
    }
  }

  // Remove whitespace characters from either (or both) ends of the string [copy]
  inline std::string StripWhitespaceFromBegining_Copy(std::string s)
  {
    StripWhitespaceFromBegining(s);
    return s;
  }
  inline std::string StripWhitespaceFromEnd_Copy(std::string s)
  {
    StripWhitespaceFromEnd(s);
    return s;
  }
  inline std::string StripWhitespace_Copy(std::string s)
  {
    StripWhitespace(s);
    return s;
  }

  // Remove comments from the end of the string.
  // The following sub strings are comment markers: "//", "#", ";"
  inline void StripComments(std::string &s)
  {
    size_t p = s.find("//");
    p = std::min<size_t>(p, s.find(";"));
    p = std::min<size_t>(p, s.find("#"));
    if (p != s.npos)
    {
      s.erase(s.begin() + p, s.end());
    }
  }

  // Split the string into two strings at the first matching character
  inline std::pair<std::string, std::string> SplitAtFirst(const std::string &s, char delim)
  {
    const size_t equalsPos = s.find_first_of(delim);
    const std::string k = equalsPos == 0 ? "" : s.substr(0, equalsPos);
    const std::string v = equalsPos == s.npos ? "" : s.substr(equalsPos + 1);
    return std::pair<std::string, std::string>(k, v);
  }

  // Split the string to substrings using any of the given delimiter characters.
  // Will write into any (most?) STL container.
  // Boost has a function that basically does this, but we avoid including Boost for another day...
  // Source: http://stackoverflow.com/questions/236129/split-a-string-in-c
  template <class ContainerT>
  void SplitString(const std::string &in, ContainerT &out, const std::string &delims = " ", bool trimEmpty = true)
  {
    std::string::size_type pos, lastPos = 0;
    while (true)
    {
      pos = in.find_first_of(delims, lastPos);
      if (pos == std::string::npos)
      {
        pos = in.length();
        if (pos != lastPos || !trimEmpty)
        {
          out.push_back(typename ContainerT::value_type(in.data() + lastPos, typename ContainerT::size_type(pos) - lastPos));
        }
        break;
      }
      else
      {
        if (pos != lastPos || !trimEmpty)
        {
          out.push_back(typename ContainerT::value_type(in.data() + lastPos, typename ContainerT::size_type(pos) - lastPos));
        }
      }
      lastPos = pos + 1;
    }
  }

  inline void OnEachToken(const std::string &in, std::function<void(std::string)> doFunction, const std::string &delims = " ")
  {
    std::size_t current, previous = 0;
    current = in.find_first_of(delims);
    while (current != std::string::npos)
    {
      doFunction(in.substr(previous, current - previous));
      previous = current + 1;
      current = in.find_first_of(delims, previous);
    }

    doFunction(in.substr(previous, current - previous));
  }

  // Case-sensitive string comparison using '*' and '?' characters in the match string as wildcards.
  // source: http://www.codeproject.com/Articles/188256/A-Simple-Wildcard-Matching-Function  (WildcardMatch_straight in a comment; avoids the recursion of the original)
  inline bool WildcardMatch(const char *input, const char *match)
  {
    assert(input && "WildcardMatch requires non-NULL input string");
    assert(match && "WildcardMatch requires non-NULL match string");

    const char *mp = NULL;
    const char *cp = NULL;

    while (*input)
    {
      if (*match == '*')
      {
        if (!*++match)
        {
          return true;
        }

        mp = match;
        cp = input + 1;
      }
      else if (*match == '?' || *match == *input)
      {
        match++;
        input++;
      }
      else if (!cp)
      {
        return false;
      }
      else
      {
        match = mp;
        input = cp++;
      }
    }

    while (*match == '*')
    {
      match++;
    }

    return !*match;
  }

  // Searches for "<key>=" and returns following word. Considers bracketed values such as "<key>=(<value1>, <value2>)"
  // Returns empty string if key not found in src.
  // Case insensitive comparison.
  // I am sure there must be a better, more established way of doing this... streams maybe?
  // Example input: "Size=(0.200, 0.100) Center=(0.50, 0.01)".  Output for "Size" is "0.200, 0.100".
  inline std::string ReadValue(const std::string &src, const std::string &key)
  {
    for (size_t i = 0; i < src.length(); ++i)
    {
      size_t equalsIdx = src.find_first_of('=', i);
      if (std::string::npos == equalsIdx)
      {
        return std::string();
      }

      std::string keyStr = src.substr(i, equalsIdx - i);
      StringUtils::StripWhitespace(keyStr);
      keyStr = StringUtils::ToLower(keyStr);

      std::string value = src.substr(equalsIdx + 1);
      StringUtils::StripWhitespace(value);
      if (!value.empty() && value.front() == '(')
      {
        size_t bracketCloseIdx = value.find_first_of(')');
        value = value.substr(0, bracketCloseIdx);
      }
      else
      {
        size_t spaceIdx = value.find_first_of(' ');
        if (std::string::npos != spaceIdx)
        {
          value = value.substr(0, spaceIdx);
        }
      }

      if (keyStr == key)
      {
        return value;
      }

      i = equalsIdx + value.length();
    }

    return std::string();
  }

  // Read all lines between the next '{' and '}'
  // Not expecting any nested blocks!
  inline std::string ReadBlock(std::vector<std::string>::const_iterator from, std::vector<std::string>::const_iterator to)
  {
    std::string block;

    for (auto it = from; it != to; ++it)
    {
      std::string line = *it;
      if (block.empty())
      {
        size_t p = line.find_first_of('{');
        if (std::string::npos != p)
        {
          block += line.substr(p + 1) + "\n";
        }
      }
      else
      {
        size_t p = line.find_first_of('}');
        if (std::string::npos != p)
        {
          block += line.substr(0, p);
          break;
        }
        else
        {
          block += line + "\n";
        }
      }
    }

    return block;
  }

  // hex is 0..9 or a..f or A..F, otherwise returns zero
  inline unsigned char HexCharToByte(const char hex)
  {
    if (hex >= '0' && hex <= '9')
    {
      return hex - '0';
    }
    else if (hex >= 'a' && hex <= 'f')
    {
      return 10 + hex - 'a';
    }
    else if (hex >= 'A' && hex <= 'F')
    {
      return 10 + hex - 'A';
    }
    return 0;
  }

  // v must be in range 0..15, otherwise returns zero
  inline char ByteToHexChar(const unsigned char v)
  {
    if (v < 10)
    {
      return v + '0';
    }
    else if (v < 16)
    {
      return v - 10 + 'A';
    }
    return '\0';
  }

  // Most significant value first. E.G. "0F" = 15 and "F0" = 240
  // Output is ordered same as input. E.G. "0FF0" = 15,240
  // Non-hex characters are treated as zeroes.
  // Precondition: even number of hex characters in hexString, otherwise final character is ignored
  inline std::vector<unsigned char> HexStringToByteArray(const std::string &hexString)
  {
    std::vector<unsigned char> ret;
    bool hiPart = true;
    unsigned char hiVal = 0;
    for (size_t i = 0; i < hexString.length(); ++i)
    {
      unsigned char b = HexCharToByte(hexString[i]);

      if (hiPart)
      {
        hiVal = b << 4;
      }
      else
      {
        ret.push_back(hiVal + b);
      }

      hiPart = !hiPart;
    }
    return ret;
  }

  // Two characters per byte, most significant value first. E.G. "0F" = 15 and "F0" = 240
  // Output is ordered same as input. E.G. 15,240 = "0FF0"
  inline std::string ByteArrayToHexString(const std::vector<unsigned char> &bytes)
  {
    std::string ret(" ", bytes.size() * 2);
    auto it = ret.begin();
    for (size_t i = 0; i < bytes.size(); ++i)
    {
      *it = ByteToHexChar((bytes[i] & 0xF0) >> 4);
      ++it;
      *it = ByteToHexChar(bytes[i] & 0x0F);
      ++it;
    }
    return ret;
  }
}

