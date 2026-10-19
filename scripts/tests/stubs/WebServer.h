#pragma once
#include <Arduino.h>
#include <map>
constexpr int HTTP_GET = 0, HTTP_POST = 1;
class WebServer {
public:
  std::map<std::string, std::string> args;
  explicit WebServer(uint16_t) {}
  String arg(const char *name) const {
    const auto it = args.find(name);
    return it == args.end() ? String{} : String(it->second.c_str());
  }
  void send(int, const char *, const char *) {}
  template<class Handler> void on(const char *, int, Handler) {}
  template<class Handler> void onNotFound(Handler) {}
  void begin() {}
  void handleClient() {}
};
