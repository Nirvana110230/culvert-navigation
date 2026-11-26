#include "checked_session.hpp"
#include <iostream>
#include <sstream>

// A CTest-only transport exercises the public CheckedSession on the real core.
int main() {
  culvert::CheckedSession session;
  Json::CharReaderBuilder reader;
  Json::StreamWriterBuilder writer;
  writer["indentation"]="";
  std::string line;
  while (std::getline(std::cin,line)) {
    Json::Value request;
    std::string errors;
    std::istringstream input(line);
    if (!Json::parseFromStream(reader,input,&request,&errors)) return 2;
    std::cout << Json::writeString(writer,session.handle(request)) << '\n' << std::flush;
  }
}
