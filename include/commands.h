#ifndef COMMANDS_H
#define COMMANDS_H

/************************************************************************************************
* Commands
*/
// typically sent from station -> capsule
#define CMDID_IR_ON             41       // power on iridium
#define CMDID_IR_OFF            42       // power off iridium
#define CMDID_IR_BP             43       // build packet
#define CMDID_IR_SP             44       // send packet
#define CMDID_DEPLOY_DROGUE     68       // trigger C02 servo 'D'
#define CMDID_FIRE_PYRO         67       // power the pyro cutters 'C'


#define CMDID_ACK               100      // acknowledge previous command


#endif
