#include "main.h"
#include "node.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "usart.h"

static connectionNode *headNode = NULL;

void addNode(uint8_t address, char *name, uint8_t len)
{
	if (inList(address))
	{
		return;
	}

	connectionNode *newNode = (connectionNode *) pvPortMalloc(sizeof(connectionNode));
	memcpy(newNode->name, name, len + 1);
	newNode->address = address;
	newNode->timer = TIME_TO_LIVE;
	newNode->online = 1;
	newNode->next = NULL;

	connectionNode **cur = &headNode;
	if (*cur == NULL)
	{
		*cur = newNode;
	}
	else
	{
		while (*cur != NULL)
		{
			cur = (connectionNode **) &((*cur)->next);
		}

		*cur = newNode;
	}
}

connectionNode* getNode(uint8_t address)
{
	connectionNode* retVal = NULL;

	connectionNode** cur = &headNode;
	while (*cur != NULL)
	{
		if ((*cur)->address == address)
		{
			retVal = *cur;
			break;
		}

		cur = (connectionNode **) &((*cur)->next);
	}

	return retVal;
}

void removeOfflineNodes(void)
{
    connectionNode **cur = &headNode;
    connectionNode *temp;

    while (*cur != NULL)
    {
        if ((*cur)->timer == 0)
        {
            temp = *cur;
            *cur = (connectionNode *) (*cur)->next;  // bypass the node
            vPortFree(temp);
        }
        else
        {
            cur = (connectionNode **) &((*cur)->next);
        }
    }
}

void refreshNode(uint8_t address)
{
    connectionNode *cur = headNode;
    while (cur != NULL)
    {
        if (cur->address == address && cur->online == 1)
        {
            cur->timer = TIME_TO_LIVE; // reset timer to initial value
            break;
        }
        cur = (connectionNode *) cur->next;
    }
}

void decrementTimers(void)
{
	connectionNode *cur = headNode;
	while(cur != NULL)
	{
		cur->timer -= 1;
		cur = (connectionNode *) cur->next;
	}
}

int inList(uint8_t address)
{
	int found = 0;
	connectionNode *cur = headNode;
	while(cur != NULL)
	{
		if (cur->address == address && cur->online == 1)
		{
			found = 1;
			break;
		}

		cur = (connectionNode *) cur->next;
	}
	return found;
}

void printList(void)
{
	char userBuf[50] = {0};
	snprintf(userBuf, sizeof(userBuf), "\tYOUR USERNAME: %s\r\n\r\n", username);
	UART_SendString(userBuf);

	connectionNode *cur = headNode;
	while (cur != NULL)
	{
		char buffer[64] = {0};
		snprintf(buffer, sizeof(buffer), "\tName: %s, Address: %u, TTL: %d\r\n", cur->name, cur->address, cur->timer);
		UART_SendString((const char *) buffer);
		cur = (connectionNode *) cur->next;
	}
}

void freeList(void)
{
	connectionNode *cur = headNode;
	connectionNode *prev = NULL;
	while(cur != NULL)
	{
		prev = cur;
		cur = (connectionNode *) cur->next;
		vPortFree(prev);
	}

	headNode = NULL;
}
