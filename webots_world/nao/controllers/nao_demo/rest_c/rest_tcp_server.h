#pragma once
#include "restapi.h"
#include "restapi_type.h"
#include "global_variable.h"

extern REST_DATA global_parse_data;


void error_message(int line) {
  printf("ERROR: LINE %d", line);
  exit(1);
}

void handle_client(int sock, char *input_data, char *output_data) {
  char buf[300];
  int mes_size;

  if ((mes_size = recv(sock, buf, 300, 0)) < 0) error_message(__LINE__);
  memcpy(input_data, buf, mes_size);
  input_data[mes_size] = '\0';

  printf("update ...\n");
  parse_request(input_data, &global_parse_data);


  while (mes_size > 0) {
    if (send(sock, output_data, strlen(output_data), 0) != strlen(output_data))
      error_message(__LINE__);
    if ((mes_size = recv(sock, buf, 300, 0)) < 0) error_message(__LINE__);

  }

  close(sock);
}

void init_restful_server_socket(restful_server_socket *_restful_server_socket) {

  if ((_restful_server_socket->server_sock =
           socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    error_message(__LINE__);

  _restful_server_socket->server.sin_family = AF_INET;
  _restful_server_socket->server.sin_addr.s_addr = htonl(INADDR_ANY);
  _restful_server_socket->server.sin_port = htons(_restful_server_socket->port);

  if (bind(_restful_server_socket->server_sock,
           (struct sockaddr *)&_restful_server_socket->server,
           sizeof(_restful_server_socket->server)) < 0)
    error_message(__LINE__);

  if (listen(_restful_server_socket->server_sock, 5) < 0) error_message(__LINE__);

  while (1) {
    printf("waiting for client...\n");
    int size = sizeof(_restful_server_socket->client);
    if ((_restful_server_socket->client_sock =
             accept(_restful_server_socket->server_sock,
                    (struct sockaddr *)&_restful_server_socket->client, &size)) < 0)
      error_message(__LINE__);

    handle_client(_restful_server_socket->client_sock,
                  _restful_server_socket->in_out_data.input_data,
                  _restful_server_socket->in_out_data.output_data);
  }
}

void update_output_data(char* output_data, restful_server_socket *_restful_server_socket) {
  strcpy(output_data, _restful_server_socket->in_out_data.output_data);
  return;
}

char* get_input_data(restful_server_socket *_restful_server_socket) {
  return _restful_server_socket->in_out_data.input_data;
}

void create_restful_server_socket(restful_server_socket *_restful_server_socket, int port) {
  _restful_server_socket->port = port;

  pthread_t thread;
  pthread_create(&thread, NULL, (void *)init_restful_server_socket, _restful_server_socket);
  pthread_detach(thread);
  strcpy(_restful_server_socket->in_out_data.output_data, " ");
  strcpy(_restful_server_socket->in_out_data.input_data, " ");
}
