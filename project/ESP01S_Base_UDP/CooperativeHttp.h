#pragma once
#include <ESP8266WiFi.h>

// Serve at most one small chunk per poll so slow clients cannot monopolize
// the same loop that forwards motor-control frames.
class CooperativeHttp {
public:
  using Handler = void (*)(const char *method, const char *path);
  explicit CooperativeHttp(uint16_t port) : listener(port) {}
  void begin() { listener.begin(); }

  void poll(Handler handler) {
    if (!open) {
      client=listener.accept();
      if (!client) return;
      open=true;
      acceptedAt=millis();
      requestLength=headerBytes=lineBytes=0;
      firstLine=true;
      sending=false;
      client.setSync(false);
      client.setNoDelay(true);
      client.setTimeout(2);
    }
    if (!client.connected() || millis()-acceptedAt>=450U) {
      close();
      return;
    }
    if (sending) {
      sendChunk();
      return;
    }
    for (unsigned budget=256; budget && client.available(); --budget) {
      int c=client.read();
      if (c<0) break;
      if (++headerBytes>4096) {
        send(431,"text/plain","Request headers too large");
        return;
      }
      if (c=='\r') continue;
      if (c!='\n') {
        ++lineBytes;
        if (firstLine) {
          if (requestLength==sizeof(request)-1 || c<32 || c>126) {
            send(400,"text/plain","Invalid request line");
            return;
          }
          request[requestLength++]=static_cast<char>(c);
        }
        continue;
      }
      if (firstLine) {
        request[requestLength]=0;
        firstLine=false;
      } else if (!lineBytes) {
        dispatch(handler);
        return;
      }
      lineBytes=0;
    }
  }

  String arg(const char *key) const {
    const size_t size=strlen(key);
    for (const char *part=query; part && *part;) {
      const char *end=strchr(part,'&');
      if (!end) end=part+strlen(part);
      if (size_t(end-part)>size && !strncmp(part,key,size) && part[size]=='=') {
        const size_t length=end-part-size-1;
        if (length>32) return String();
        char value[33];
        memcpy(value,part+size+1,length);
        value[length]=0;
        return String(value);
      }
      part=*end ? end+1 : end;
    }
    return String();
  }

  char *responseBuffer() { return response; }
  static constexpr size_t responseCapacity=4800;

  void send(unsigned code,const char *type,const char *data) {
    size_t length=strlen(data);
    if (length>=sizeof(response)) {
      code=500;
      type="text/plain";
      data="Response too large";
      length=strlen(data);
    }
    if (response!=data) memcpy(response,data,length);
    prepare(code,type,response,length,false);
  }
  void send(unsigned code,const char *type,const String &data) { send(code,type,data.c_str()); }
  void sendPage(const char *page,size_t length) { prepare(200,"text/html; charset=utf-8",page,length,true); }

private:
  WiFiServer listener;
  WiFiClient client;
  bool open=false, firstLine=true, sending=false, flash=false;
  uint32_t acceptedAt=0;
  uint16_t requestLength=0, headerBytes=0, lineBytes=0;
  char request[256], header[512], response[responseCapacity];
  const char *query=nullptr, *body=nullptr;
  size_t headerLength=0, bodyLength=0, sent=0;

  void close() {
    // stop(0) uses the core's default flush timeout.
    client.stop(1);
    client=WiFiClient();
    open=false;
  }

  void dispatch(Handler handler) {
    char *target=strchr(request,' ');
    if (!target) { send(400,"text/plain","Bad request"); return; }
    *target++=0;
    char *version=strchr(target,' ');
    if (!version) { send(400,"text/plain","Bad request"); return; }
    *version++=0;
    if (*target!='/' || (strcmp(version,"HTTP/1.1") && strcmp(version,"HTTP/1.0"))) {
      send(400,"text/plain","Bad request");
      return;
    }
    char *args=strchr(target,'?');
    if (args) *args++=0;
    query=args;
    handler(request,target);
    query=nullptr;
  }

  void prepare(unsigned code,const char *type,const char *data,size_t length,bool fromFlash) {
    const char *reason="OK";
    switch (code) {
      case 202: reason="Accepted"; break;
      case 204: reason="No Content"; break;
      case 400: reason="Bad Request"; break;
      case 404: reason="Not Found"; break;
      case 410: reason="Gone"; break;
      case 431: reason="Request Header Fields Too Large"; break;
      case 500: reason="Internal Server Error"; break;
      case 503: reason="Service Unavailable"; break;
    }
    const int n=snprintf(header,sizeof(header),
      "HTTP/1.1 %u %s\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
      "Connection: close\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\n"
      "Access-Control-Allow-Private-Network: true\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
      "Access-Control-Allow-Headers: Content-Type, X-Route-Token\r\nAccess-Control-Max-Age: 600\r\n\r\n",
      code,reason,type,static_cast<unsigned>(length));
    if (n<0 || size_t(n)>=sizeof(header)) { close(); return; }
    headerLength=n;
    body=data;
    bodyLength=length;
    flash=fromFlash;
    sent=0;
    sending=true;
  }

  void sendChunk() {
    const int writable=client.availableForWrite();
    if (writable<=0) return;
    const bool headerPending=sent<headerLength;
    const size_t offset=headerPending ? sent : sent-headerLength;
    const size_t remaining=(headerPending ? headerLength : bodyLength)-offset;
    if (!remaining) { close(); return; }
    size_t count=remaining<512 ? remaining : 512;
    if (count>size_t(writable)) count=writable;
    const char *data=(headerPending ? header : body)+offset;
    char chunk[512];
    if (!headerPending && flash) {
      memcpy_P(chunk,data,count);
      data=chunk;
    }
    sent+=client.write(reinterpret_cast<const uint8_t *>(data),count);
    if (sent==headerLength+bodyLength) close();
  }
};
