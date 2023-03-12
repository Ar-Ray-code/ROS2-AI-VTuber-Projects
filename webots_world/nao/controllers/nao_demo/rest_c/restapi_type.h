#pragma once

#include <arpa/inet.h>
#include <memory.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <sys/socket.h>
// #include <unistd.h>

typedef struct restapi restapi;
typedef struct restful_server_socket restful_server_socket;

// (tcp_data) ========================================
// - input_data: data from client
// - output_data: data to client
// =================================================
struct tcp_data {
  char input_data[300];
  char output_data[300];
};

// (restful_server_socket) ===============================
// description: socket for server
// - port: port number
// - client: client socket
// - server: server socket
// - server_sock: server socket
// - client_sock: client socket
// - in_out_data: data from client and to client (input_data, output_data)
// =================================================
typedef struct restful_server_socket {
  unsigned int port;
  struct sockaddr_in client;
  struct sockaddr_in server;
  int server_sock;
  int client_sock;

  struct tcp_data in_out_data;
} restful_server_socket;

// (restapi_method) =============================
// description: method for restful api
// - GET: get data from server
// - POST: post data to server
// - PUT: update data to server
// - DELETE: delete data from server
// =================================================
typedef enum restapi_method {
    GET = 0,
    POST,
    PUT,
    DELETE
} restapi_method;

// (restapi) ====================================
// description: restful api
// - target: target of restful api
// - method: method of restful api (GET, POST, PUT, DELETE)
// - data: data of restful api
// =================================================
typedef struct REST_DATA {
    char target[100]; // http://localhost:8080/input --> target = "input"
    restapi_method method;
    char data[300];
} REST_DATA;
