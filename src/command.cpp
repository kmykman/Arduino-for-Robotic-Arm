#include "command.h"

Command::Command() {
  //initialize Command to a zero-move value;
  command.valueX = NAN;
  command.valueY = NAN;
  command.valueZ = NAN;
  command.valueF = 0;
  command.valueE = 0;
  command.valueT = 0;
  command.valueP = -1;

  message = "";
}

bool Command::handleGcode() {
  char c;
  bool isDataAval = false;

  if (BlueTooth.available()) {
    c = BlueTooth.read();
    //Console.println(c, DEC);
    isDataAval = true;
  } else if (Console.available()) {
    c = Console.read();
    isDataAval = true;
  }

  if (isDataAval) {
    if (c == '\n') {
      return false;
    }
    if (c == '\r') {
      bool b = processMessage(message);
      message = "";
      return b;
    } else {
      message += c;
    }
  }
  return false;
}

bool Command::processMessage(String& msg) {
  int done = command.valueP;

  msg += ' ';  //helps parsing
  command.id = msg[0];
  if (msg.indexOf('P') == -1){
    Console.println("--> no pck no.");
    printErr();
    return false;
  }
  //exit if not GCode
  if ((command.id != 'G') && (command.id != 'M')) {
    printErr();
    return false;
  }

    int first = 1;
    int last = pos(msg, ' ', 1);
    if (last < 0) {
      printErr();
      return false;
    }
    String s = msg.substring(first, last);
    command.num = s.toInt();


    //parse up to 5 Values
    command.valueX = NAN;
    command.valueY = NAN;
    command.valueZ = NAN;
    command.valueE = NAN;
    command.valueF = 0;
    command.valueT = 0;
    command.valueP = -1;
    int parsePosition = last + 1;
    int i = 0;
    while (i < 5) {
      char id = msg[parsePosition++];
      if (id != ' ') {  //test if a command here
        int first = parsePosition;
        int last = pos(msg, ' ', parsePosition);
        if (last < first) { //test if String is valid
          i = 5; //exit;
        } else {
          String floatString = msg.substring(first, last);  //should contain a Numeric value
          float value = floatString.toFloat();
          switch (id) {
            case 'X': command.valueX = value; break;
            case 'Y': command.valueY = value; break;
            case 'Z': command.valueZ = value; break;
            case 'E': command.valueZ = value; break;
            case 'F': command.valueF = value; break;
            case 'T': command.valueT = value; break;
            case 'P': 
                command.valueP = floor(value);
              if (done == floor(value)){
                Serial1.print("A" + String(getACKno()));
                Console.println("--> Resend A" + String(getACKno()));
                return false;
              }
              break;
            default: i = 5;
          }
          parsePosition = last + 1;
        }
      } else {
        i = 5; //exit;
      }
      i++;
    }
  // }

  return true;
}

Cmd Command::getCmd() const {
  return command;
}

int Command::pos(String& s, char c, int start) {
  int len = s.length();
  for (int i = start; i < len; i++) {
    if (c == s[i]) {
      return i;
    }
  }
  return -1;
}

int Command::getACKno() const {
  return command.valueP;
}

void printErr() {
  // Console.println("rs"); //'resend'
  Serial1.print("F");
}

void printFault() {
  Console.println("!!");
  Serial1.print("F");
}

void printComment(char* c) {
  Console.print("// ");
  Console.println(c);
}

void printComment(String& s) {
  Console.print("// ");
  Console.println(s);
}

void printOk() {
  // Console.println("ok");
  Serial1.print("ok");
}
