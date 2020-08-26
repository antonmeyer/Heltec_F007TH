/* calculation for humidity 
0,1% precise formula:
absHum = (6.112 × e^[(17.67 × T)/(T+243.5)] × rh × 2.1674) / (273.15 + T)
in g/m3

so the trick could be calculate both absHum and open the window if basement is higer
+ a margin
*/

#include <math.h>

float absHum (float T, char rh) {

return ((6.112 * exp(((17.67 *T)/(T+243.5))) * rh * 2.1674) / (273.15 + T));
}
                                                                            

