


#include "stasm.hpp"

bool fgErr;                         // set if there is an error used by Windows
static char sgErr[MAX_PRINT_LEN];   // general purpose error msg buffer

//-----------------------------------------------------------------------------
// Print an error message and exit.
// This puts a \n on the msg so you shouldn't.

void Err (const char *pArgs, ...)       // args like printf
{
fgErr = true;                           // set global error flag

va_list pArg;
va_start(pArg, pArgs);
vsprintf(sgErr, pArgs, pArg);
va_end(pArg);

lprintf("\nError: %s\n", sgErr);
fflush(stdout);
if (pgLogFile)
    fflush(pgLogFile);

#if _DEBUG
if (ENTER_DEBUGGER_ON_ERR)
    ENTER_DEBUGGER("ENTER_DEBUGGER_ON_ERR is true "
                   "so forcing entry to debugger");
#endif

Shutdown();
exit(1);
}

//-----------------------------------------------------------------------------
// Print an warning message with standard "Warning" prefix
// Using a standard format allows easy post processing of
// log file: grep for "Warning:"
// This puts a \n on the msg so you shouldn't.

void Warn (const char *pArgs, ...)              // args like printf
{
va_list pArg;
va_start(pArg, pArgs);
vsprintf(sgErr, pArgs, pArg);
va_end(pArg);
lprintf("Warning: %s\n", sgErr);
}

//-----------------------------------------------------------------------------
// same as Warn() but prefix a new line

void WarnWithNewLine (const char *pArgs, ...)   // args like printf
{
lprintf("\n");
va_list pArg;
va_start(pArg, pArgs);
vsprintf(sgErr, pArgs, pArg);
va_end(pArg);
lprintf("Warning: %s\n", sgErr);
}

