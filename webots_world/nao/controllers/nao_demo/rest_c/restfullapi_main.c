#include "restapi.h"

char data[300] = "POST /input HTTP/1.1\nHost: localhost:5000\nUser-Agent: curl/7.81.0\nAccept: */*\nContent-Type: application/json\nContent-Length: 19\n\n{\"message\":\"hello\"}";

int main() {
    REST_DATA r;
    parse_request(data, &r);
    printf("method: %d \n", r.method);
    printf("target: %s \n", r.target);
    printf("data: %s \n", r.data);
    return 0;
}
