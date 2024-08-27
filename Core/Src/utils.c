#include "utils.h"
#include "main.h"
#include "node.h"
#include "usart.h"

void printUsage(void)
{
	UART_SendString("Usage:\r\n\tClear screen: <clear>\r\n\tSee who's online: <%l>\r\n\tSend DM: <%m> <address in decimal> <message>\r\n\tSend broadcast: <%b> <message>\r\n\t");
	UART_SendString(red_text);
	UART_SendString("Broadcasts printed in red\r\n\t");
	UART_SendString(green_text);
	UART_SendString("DM's printed in green\r\n\r\n");
	UART_SendString(reset_text);
}

void getUsername(void)
{
	// Prompt for username
	UART_SendString("Welcome to Chat! Please enter a username: ");

	// Block until username is set
	while(!loggedOn);
	UART_SendString(clear_screen);
	UART_SendString(move_cursor_home);
	printUsage();
	UART_SendString("$: ");
}

uint16_t createPDU(uint8_t flag, uint8_t *buf, uint8_t *message, uint16_t len)
{
	uint16_t bytesToSend = 0;

	memcpy(buf, &flag, 1);
	bytesToSend++;
	memcpy(buf + bytesToSend, username, strlen(username));
	bytesToSend += strlen(username);
	memcpy(buf + bytesToSend, "\0", 1);
	bytesToSend++;

	if (flag == MESSAGE)
	{
		memcpy(buf + bytesToSend, message, len);
		bytesToSend += len;
		memcpy(buf + bytesToSend, "\0", 1);
		bytesToSend++;
	}

	return bytesToSend;
}

void processInput(message_t *msg_t)
{
	if (strncasecmp(msg_t->message, "clear", 5) == 0)
	{
		UART_SendString(move_cursor_home);
		UART_SendString(clear_screen);
		printUsage();
	}
	else if (strncasecmp(msg_t->message, "%L", 2) == 0)
	{
		printList();
	}
	else if (strncasecmp(msg_t->message, "%M", 2) == 0 || strncasecmp(msg_t->message, "%B", 2) ==0)
	{
		uint8_t destAddress = 0;
		char buf[MAXBUF] = {0};
		if (strncasecmp(msg_t->message, "%M", 2) == 0)
		{
			parseCommand(msg_t->message, &destAddress, buf);
		}
		else if (strncasecmp(msg_t->message, "%B", 2) == 0)
		{
			destAddress = BROADCAST;
			memcpy(buf, msg_t->message + 3, msg_t ->len);
		}

		// Transmit Message
		uint8_t sendBuf[MAXBUF] = {0};
		uint16_t bytesToSend = createPDU(MESSAGE, (uint8_t *) sendBuf, (uint8_t *) buf, strlen(buf));

		SpiritCmdStrobeSabort();
		SpiritPktStackSetDestinationAddress(destAddress);
		xTxDoneFlag = S_RESET;
		SpiritCmdStrobeFlushTxFifo();
		SPSGRF_StartTx((uint8_t *) sendBuf, bytesToSend);
		while(!xTxDoneFlag) {};
		SpiritCmdStrobeRx();
	}
}

void processPayload(uint8_t srcAddress, uint8_t destAddress, char* payload, uint16_t bytesRecv)
{
	if (payload[0] == LOG_ON)
	{
		// Get username from person logging on, add them to Online List
		char recvUser[21] = {'\0'};
		uint8_t recvUserLen = strlen(payload + 1);
		memcpy(recvUser, payload + 1, recvUserLen);
		addNode(srcAddress, recvUser, recvUserLen);

		 SpiritPktStackSetDestinationAddress(srcAddress);

		// Reply with Announcement Packet
		uint8_t payload[22] = {0};
		uint16_t bytesToSend = createPDU(RESPONSE, (uint8_t *) payload, NULL, 0);
	    xTxDoneFlag = S_RESET;
	    SpiritCmdStrobeFlushTxFifo();
	    SPSGRF_StartTx(payload, bytesToSend);
	    while(!xTxDoneFlag);

	   SpiritCmdStrobeRx();
	}
	else if (payload[0] == RESPONSE) // process announcement packet from others
	{
		char recvUser[21] = {'\0'};
		uint8_t recvUserLen = strlen(payload + 1);
		memcpy(recvUser, payload + 1, recvUserLen);
		addNode(srcAddress, recvUser, recvUserLen);
	}
	else if (payload[0] == HEARTBEAT)
	{
		if (inList(srcAddress)) // If already online, refresh TTL
		{
			refreshNode(srcAddress);
		}
		else // If not online, add to online list (in case announcement packet or log on was missed)
		{
			char recvUser[21] = {'\0'};
			uint8_t recvUserLen = strlen(payload + 1);
			memcpy(recvUser, payload + 1, recvUserLen);
			addNode(srcAddress, recvUser, recvUserLen);
		}
	}
	else if (payload[0] == MESSAGE) // print message
	{
		if (inList(srcAddress)) // If already online, refresh TTL
		{
			refreshNode(srcAddress);
		}
		else // If not online, add to online list (in case announcement packet or log on was missed)
		{
			char recvUser[21] = {'\0'};
			uint8_t recvUserLen = strlen(payload + 1);
			memcpy(recvUser, payload + 1, recvUserLen);
			addNode(srcAddress, recvUser, recvUserLen);
		}

		printPayload(srcAddress, destAddress, payload, bytesRecv);
	}
}

void printPayload(uint8_t srcAddress, uint8_t destAddress, char* payload, uint16_t  bytesRecv)
{
	if (inList(srcAddress))
	{
		// Get node
		connectionNode *cur = getNode(srcAddress);
		char buf[MAXBUF] = {0};
		UART_SendString("\r\n");

		if (destAddress == BROADCAST)
		{
			UART_SendString(red_text);
			snprintf(buf, sizeof(buf), "%s (%u): %s\r\n", cur->name, srcAddress, payload + 1 + strlen(cur->name) + 1);
			UART_SendString(buf);
			UART_SendString(reset_text);
		}
		else if (destAddress == MY_ADDRESS)
		{
			UART_SendString(green_text);
			snprintf(buf, sizeof(buf), "%s (%u): %s\r\n", cur->name, srcAddress, payload + 1 + strlen(cur->name) + 1);
			UART_SendString(buf);
			UART_SendString(reset_text);
		}

		UART_SendString("$: ");
	}
}

void parseCommand(const char *input, uint8_t *number, char *message) {
    const char *ptr = input;

    ptr += 3; // skip past command (assume perfect input)

    // get the address
    uint16_t tempNumber = 0;
    while (isdigit((unsigned char)*ptr)) {
        tempNumber = tempNumber * 10 + (*ptr - '0');
        ptr++;
    }

    *number = (uint8_t)tempNumber;

    // skip the space after the address
    if (*ptr == ' ') {
        ptr++;
    }

    // copy the remaining message
    size_t i = 0;
    while (*ptr != '\0') {
        message[i++] = *ptr++;
    }
    message[i] = '\0';  // null-terminate the message
}
