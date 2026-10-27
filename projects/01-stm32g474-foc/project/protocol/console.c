#include "console.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

/** @brief 见 console.h。 */
void
console_init(console_t *console)
{
    memset(console, 0, sizeof(*console));
}

/** @brief 见 console.h。 */
bool
console_feed(console_t *console, uint16_t token, console_handler_t handler, char *response)
{
    unsigned char ch = (unsigned char)token;
    if ((token & 0x100U) != 0U) { console->invalid = true; }
    if (ch == '\n')
    {
        char *argv[8];
        unsigned argc = 0U;
        bool extra = false;
        console->line[console->length] = '\0';
        if (console->invalid) { strcpy(response, "ERR LINE CORRUPT"); }
        else if (console->too_long) { strcpy(response, "ERR LINE TOO_LONG"); }
        else
        {
            char *cursor = console->line;
            while (*cursor != '\0')
            {
                while (*cursor == ' ' || *cursor == '\t') { ++cursor; }
                if (*cursor == '\0') { break; }
                if (argc == sizeof(argv)/sizeof(argv[0])) { extra = true; break; }
                argv[argc++] = cursor;
                while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') { ++cursor; }
                if (*cursor != '\0') { *cursor++ = '\0'; }
            }
            if (argc == 0U || extra) { strcpy(response, "ERR LINE ARGUMENT"); }
            else { handler(argc, argv, response, CONSOLE_RESPONSE_MAX); }
        }
        size_t length = strlen(response);
        if (length > CONSOLE_RESPONSE_MAX - 3U)
        {
            strcpy(response, "ERR LINE INTERNAL");
            length = strlen(response);
        }
        memcpy(response + length, "\r\n", 3U);
        console_init(console);
        return true;
    }
    /* CR 只允许作为尾部字符，后面出现其他正文则该行无效。 */
    if (ch == '\r')
    {
        if (console->length <= CONSOLE_LINE_MAX && !console->too_long)
        {
            console->line[console->length] = '\r';
        }
        return false;
    }
    if (console->line[console->length] == '\r') { console->invalid = true; }
    if ((ch < 32U && ch != '\t') || ch > 126U) { console->invalid = true; }
    if (console->length == CONSOLE_LINE_MAX) { console->too_long = true; return false; }
    console->line[console->length++] = (char)toupper(ch);
    return false;
}
