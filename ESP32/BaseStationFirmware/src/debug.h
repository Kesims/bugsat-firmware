#ifndef BASESTATIONFIRMWARE_DEBUG_H
#define BASESTATIONFIRMWARE_DEBUG_H

#define DEBUG 1

#if DEBUG == 1
#define debugPrint(x) Serial.print(x)
#define debugPrintln(x) Serial.println(x)
#else
#define debugPrint(x)

#define debugPrintln(x)
#endif


#endif //BASESTATIONFIRMWARE_DEBUG_H
