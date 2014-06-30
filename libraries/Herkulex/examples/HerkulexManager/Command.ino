
void printIntro()
{
  Serial.println("Herkulex Manager");
  printPrompt();
}

void printPrompt()
{
  Serial.print("> ");
}

void printCmdError()
{
  Serial.println("Bad command. type 'help'");
  printPrompt();
}

void printMaxLengthError()
{
  Serial.println();
  Serial.println("Max command length");
  printPrompt();
}

void cmdHelp()
{
  Serial.println("========= Command ===========");
  Serial.println(" help");
  Serial.println(" scan");
  Serial.println(" id [old] [new]");
  Serial.println(" go [id] [pos]");
  Serial.println(" read [id]");
  Serial.println("=============================");
  Serial.println();
  printPrompt();
}

void cmdScan()
{
  int count = 0;
  Serial.println("Start scaning id...");
  for(int id=0; id<BROADCAST_ID; id++)
  {
    int err = Herkulex.stat(id);
    if(err >= 0)
    {
      Serial.print("[");
      Serial.print(id, DEC);
      Serial.print(": ");
      Serial.print(err, DEC);
      Serial.print("] ");
      count++;
      if(err > 0)
        Herkulex.clearError(id);
    }
  }
  Serial.println();
  Serial.print("Total found number is ");
  Serial.println(count, DEC);
  printPrompt();
}

void cmdGo(char* arg1, char* arg2)
{
  if(arg1 == 0)
    Serial.println("Invalid id");
  else
  {
    int id = atoi(arg1);
    if(arg2 == 0)
    {
      Herkulex.torqueON(BROADCAST_ID);
      Herkulex.moveOne(BROADCAST_ID, id, 1, 0);
    }
    else
    {
      int pos = atoi(arg2);
      Herkulex.torqueON(id);
      Herkulex.moveOne(id, pos, 1, 0);
    }
  }
  printPrompt();
}

void cmdID(char* arg1, char* arg2)
{
  if(arg1 == 0)
    Serial.println("Invalid id");
  else
  {
    int id = atoi(arg1);
    if(arg2 == 0)
      Serial.println("Invalid value");
    else
    {
      int value = atoi(arg2);
      if(value >= 0 && value < BROADCAST_ID)
      {
        Herkulex.set_ID(id, value);
        Herkulex.reboot(id);
      }
      else
        Serial.println("Invalid value");      
    }
  }
  printPrompt();
}

void cmdRead(char* arg1)
{
  if(arg1 == 0)
    Serial.println("Invalid id");
  else
  {
    int id = atoi(arg1);
    if(id >= 0 && id <= BROADCAST_ID)
    {
      int pos = Herkulex.getPosition(id);
      if(pos >= 0)
        Serial.println(pos, DEC);
    }
    else
      Serial.println("Invalid id");
  }
  printPrompt();
}

