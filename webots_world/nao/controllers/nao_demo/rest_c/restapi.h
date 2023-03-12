#pragma once

#include "restapi_type.h"

// parse_request
void parse_request(char *request, REST_DATA *r)
{
    restapi_method m;
    char *tokens[10];
    int i = 0;

    // not found GET, POST, PUT, DELETE
    if (strstr(request, "GET") == NULL &&
        strstr(request, "POST") == NULL &&
        strstr(request, "PUT") == NULL &&
        strstr(request, "DELETE") == NULL)
    {
        r->method = 0;
        strcpy(r->target, "");
        strcpy(r->data, "");
        return;
    }

    char *token = strtok(request, "\n");

    while (token != NULL) {
        tokens[i] = token;
        token = strtok(NULL, "\n");
        i++;
    }

    char *method = strtok(tokens[0], " ");
    if (strcmp(method, "GET") == 0) {
        m = GET;
    } else if (strcmp(method, "POST") == 0) {
        m = POST;
    } else if (strcmp(method, "PUT") == 0) {
        m = PUT;
    } else if (strcmp(method, "DELETE") == 0) {
        m = DELETE;
    }

    char *target = strtok(NULL, " ");
    char *data = tokens[i - 1];

    strcpy(r->target, target);
    r->method = m;
    strcpy(r->data, data);
}
