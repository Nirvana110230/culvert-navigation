#include "session.hpp"
#include <iostream>

int main() {
  using namespace culvert;
  Json::CharReaderBuilder parser;
  parser["collectComments"]=false; parser["allowComments"]=false; parser["allowTrailingCommas"]=false;
  parser["failIfExtra"]=true; parser["rejectDupKeys"]=true; parser["allowSpecialFloats"]=false;
  std::unique_ptr<Json::CharReader> reader(parser.newCharReader());
  Json::StreamWriterBuilder writer;
  writer["indentation"]="";
  Session server;
  std::string line;
  while (std::getline(std::cin,line)) {
    auto started=Clock::now();
    Json::Value req,out;
    std::string errors;
    if (line.size()>(32U<<20) || !reader->parse(line.data(),line.data()+line.size(),&req,&errors)) {
      out=server.invalid_json(started);
    } else out=server.handle(req,started);
    std::cout<<Json::writeString(writer,out)<<'\n'<<std::flush;
  }
}
