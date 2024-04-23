#ifndef BASESTATIONFIRMWARE_LORA_HANDLER_H
#define BASESTATIONFIRMWARE_LORA_HANDLER_H


void check_lora_receiver();
void lora_init();
void on_lora_receive(int packetSize);

#endif //BASESTATIONFIRMWARE_LORA_HANDLER_H
