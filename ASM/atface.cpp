


#include "stasm.hpp"

//-----------------------------------------------------------------------------
char *sGetAtFaceString (unsigned Attr)
{
static char s[SLEN];

s[0] = 0;
unsigned int BitMask = 0x8000;
int iAttr = 16;

while (iAttr--)
    {
    if ((Attr & BitMask) && sgFaceAttr[iAttr][0])
        {
        if (s[0])
            strcat(s, ",");
        strcat(s, sgFaceAttr[iAttr]);
        }
    BitMask >>= 1;
    }
return s;
}

//-----------------------------------------------------------------------------
char *sGetDetString (unsigned Attr)
{
if (Attr & FA_Rowley)
    return (char *)"Rowley";

if (Attr & FA_ViolaJones)
    return (char *)"Viola Jones";

Err("sGetDetString: Illegal Attr %x", Attr);

return NULL;    // keep compiler warnings quiet
}

