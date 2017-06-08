#ifndef StringUtils_h
#define StringUtils_h

// A hack to backport the float/double to String cast to Arduino 1.0.
// This can be removed once Arduino 1.6 or greater is finally ported to ARM.
String fts(double value, unsigned char decimalPlaces=2)
{
    char buf[33];
    return String(dtostrf(value, (decimalPlaces + 2), decimalPlaces, buf));
}
/*
String floatToString(float floatVal){
    char charVal[10];
    dtostrf(floatVal, 4, 3, charVal);
    return String(charVal);
}
*/

// A hack to backport String.toFloat to Arduino 1.0.
// This can be removed once Arduino 1.6 or greater is finally ported to ARM.
// http://forum.arduino.cc/index.php?topic=45357.0
float stf(String readstring){
    char carray[readstring.length() + 1]; //determine size of the array
    readstring.toCharArray(carray, sizeof(carray)); //put readStringinto an array
    return atof(carray); //convert the array into an Integer
}

#endif