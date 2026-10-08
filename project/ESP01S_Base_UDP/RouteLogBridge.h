/* V4.7: bounded, read-only journal bridge. No wait loops, no motor commands.
 * Only a complete CRC-checked batch is exposed to HTTP readers.
 */
uint32_t journalClient=0,journalRequest=0,journalAfter=0,journalAt=0;
uint32_t journalBootClient=0,journalBootRequest=0,journalOldest=0,journalNewest=0,journalFirst=0;
uint16_t journalConfig=0,journalMask=0;
uint8_t journalCount=0;
bool journalPending=false,journalHeader=false,journalReady=false,journalDelivered=false;
JournalPoint journalPoints[JOURNAL_BATCH];
void parseJournalLine(const char *line){
  int64_t v[10];size_t n=strlen(line);unsigned i;
  if(!journalPending)return;
  if(JournalFields(line,n,'2',v,9)){
    for(i=0;i<9;i++)if(!JournalUnsigned(v[i]))return;
    if(v[0]!=journalClient||v[1]!=journalRequest||!v[2]||!v[3]||!v[4]||
       v[5]<v[4]||v[5]-v[4]>=JOURNAL_CAPACITY||v[6]>65535||v[7]>JOURNAL_BATCH||
       v[8]<v[4]||v[8]>v[5]+1||((v[8]+v[7]!=v[5]+1)&&v[7]!=JOURNAL_BATCH)||
       v[8]+v[7]>v[5]+1)return;
    journalBootClient=(uint32_t)v[2];journalBootRequest=(uint32_t)v[3];
    journalOldest=(uint32_t)v[4];journalNewest=(uint32_t)v[5];
    journalConfig=(uint16_t)v[6];journalCount=(uint8_t)v[7];journalFirst=(uint32_t)v[8];
    journalMask=0;journalHeader=true;journalReady=journalCount==0;return;
  }
  if(!journalHeader||!JournalFields(line,n,'3',v,10))return;
  for(i=0;i<5;i++)if(!JournalUnsigned(v[i]))return;
  if(v[0]!=journalClient||v[1]!=journalRequest||v[2]>=journalCount||v[3]!=journalFirst+v[2]||
     v[5]<-100000000||v[5]>100000000||v[6]<-100000000||v[6]>100000000||
     v[7]<-3142||v[7]>3142||v[8]<0||v[8]>1000000000||v[9]<0||v[9]>1)return;
  JournalPoint &p=journalPoints[(unsigned)v[2]];
  p.sequence=(uint32_t)v[3];p.ms=(uint32_t)v[4];p.x=(int32_t)v[5];p.y=(int32_t)v[6];
  p.h=(int32_t)v[7];p.distance=(uint32_t)v[8];p.flags=(uint32_t)v[9];
  journalMask|=(uint16_t)(1U<<(unsigned)v[2]);
  journalReady=journalMask==(1U<<journalCount)-1U;
}
void sendJournal(){
  addCorsHeaders();
  String arg=server.arg("after");const char *p=arg.c_str(),*end=p+arg.length();uint32_t after;
  if(!arg.length()||arg.length()>10||!HH_Number(&p,end,&after)||p!=end){
    server.send(400,"application/json","{\"error\":\"cursor\"}");return;
  }
  uint32_t now=millis();
  bool same=journalPending&&after==journalAfter;
  if(same&&journalReady&&now-journalAt<=1500U&&
     ((journalCount&&after!=UINT32_MAX)||now-journalAt<150U)){
    char json[1400];int n=snprintf(json,sizeof(json),
      "{\"protocol\":2,\"session\":\"%lu-%lu\",\"oldest\":%lu,\"newest\":%lu,\"configCrc\":%u,\"samples\":[",
      (unsigned long)journalBootClient,(unsigned long)journalBootRequest,
      (unsigned long)journalOldest,(unsigned long)journalNewest,journalConfig);
    if(n<0||(size_t)n>=sizeof(json)){server.send(500,"application/json","{}");return;}
    size_t used=(size_t)n;
    for(unsigned i=0;i<journalCount;i++){
      const JournalPoint &q=journalPoints[i];
      n=snprintf(json+used,sizeof(json)-used,"%s[%lu,%lu,%ld,%ld,%ld,%lu,%lu]",
        i?",":"",(unsigned long)q.sequence,(unsigned long)q.ms,(long)q.x,(long)q.y,(long)q.h,
        (unsigned long)q.distance,(unsigned long)q.flags);
      if(n<0||(size_t)n>=sizeof(json)-used){server.send(500,"application/json","{}");return;}
      used+=(size_t)n;
    }
    if(used+3>sizeof(json)){server.send(500,"application/json","{}");return;}
    strcpy(json+used,"]}");server.send(200,"application/json",json);journalDelivered=true;return;
  }
  /* One bounded in-flight read is shared by tabs; a missing UART frame retries
   * the same browser cursor with a NEW request id, not an invented pose.
   */
  /* Another tab must not replace a ready batch before its reader receives it.
   * Expiry releases the slot if that reader closes or loses its connection. */
  if(!journalPending||(journalReady&&(same||journalDelivered))||now-journalAt>1000U){
    char line[80];uint32_t next=journalRequest+1U;if(!next)next=1;
    int n=JournalFinish(line,sizeof(line),snprintf(line,sizeof(line),"J1,%lu,%lu,%lu",
      (unsigned long)journalClient,(unsigned long)next,(unsigned long)after));
    if(n<=0||Serial.availableForWrite()<n+64){server.send(503,"application/json","{\"error\":\"serial_busy\"}");return;}
    Serial.write((const uint8_t*)line,n);
    journalRequest=next;journalAfter=after;journalAt=now;journalPending=true;
    journalHeader=journalReady=journalDelivered=false;journalMask=0;
  }
  server.send(202,"application/json","{\"pending\":true}");
}
