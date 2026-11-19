/* Bounded UART bridge: HTTP 202 only means queued; Q2 is the MCU ACK. */
#include "ReturnProtocol.h"
uint32_t returnStatus[11]={0},returnAt=0;
bool returnReceived=false;
bool returnNumber(const char *key,uint32_t &value){
  String a=server.arg(key);const char *p=a.c_str(),*end=p+a.length();
  return a.length()&&a.length()<=10&&HH_Number(&p,end,&value)&&p==end;
}
void parseReturnLine(const char *line){
  uint32_t v[11];
  if(!ReturnFields(line,strlen(line),'2',v,11)||v[2]>6||v[4]>4||v[5]>9||
     v[6]>1024||v[7]>=1024||v[8]>1||v[9]>3)return;
  memcpy(returnStatus,v,sizeof(v));returnAt=millis();returnReceived=true;
}
void sendReturnStatus(){
  char json[400];const uint32_t *v=returnStatus;
  bool online=returnReceived&&millis()-returnAt<700U;
  snprintf(json,sizeof(json),
    "{\"online\":%s,\"boot\":%lu,\"ack\":%lu,\"result\":%lu,\"task\":%lu,"
    "\"state\":%lu,\"reason\":%lu,\"points\":%lu,\"index\":%lu,\"ready\":%s,"
    "\"action\":%lu,\"commandTask\":%lu}",online?"true":"false",
    (unsigned long)v[0],(unsigned long)v[1],(unsigned long)v[2],(unsigned long)v[3],
    (unsigned long)v[4],(unsigned long)v[5],(unsigned long)v[6],(unsigned long)v[7],
    online&&v[8]?"true":"false",(unsigned long)v[9],(unsigned long)v[10]);
  server.send(200,"application/json",json);
}
void sendReturnCommand(){
  String action=server.arg("action");uint32_t boot,task,request;
  bool heartbeat=action=="heartbeat";
  unsigned code=action=="set"?1:action=="start"?2:action=="stop"?3:0;
  if((!code&&!heartbeat)||!returnNumber("boot",boot)||!returnNumber("task",task)||
     !returnNumber(heartbeat?"pulse":"request",request)||!request){
    server.send(400,"application/json","{\"error\":\"invalid_return_command\"}");return;
  }
  if(!returnReceived||!boot||boot!=returnStatus[0]||millis()-returnAt>=700U){
    server.send(409,"application/json","{\"error\":\"stale_mcu\"}");return;
  }
  char line[96];int n;
  if(heartbeat)n=snprintf(line,sizeof(line),"Q3,%lu,%lu,%lu",
    (unsigned long)boot,(unsigned long)task,(unsigned long)request);
  else n=snprintf(line,sizeof(line),"Q1,%lu,%lu,%u,%lu",
    (unsigned long)boot,(unsigned long)request,code,(unsigned long)task);
  n=ReturnFinish(line,sizeof(line),n);
  if(n<=0||Serial.availableForWrite()<n+64){
    server.send(503,"application/json","{\"error\":\"serial_busy\"}");return;
  }
  Serial.write(reinterpret_cast<const uint8_t*>(line),n);
  server.send(202,"application/json","{\"pending\":true}");
}
