#include <iostream>
#include <string>

#include <cpprest/http_client.h>

#include "astar.h"

using namespace std;
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features

int main(int argc, char* argv[])
{

    if (argc != 2) {
        cout << "Usage: controller <name robot>" << endl;
        return 1;
    }

    auto astar = AStar();

    // Create http_client to send the request.
    http_client client(U("https://robomaze.skadge.org/"));


    string next_move("E");

    while(true) {


            // Build request URI and start the request.
            uri_builder builder(U("/api"));
            builder.append_query(U("move"), U("[\"" + string(argv[1]) +"\",\"" + next_move + "\"]"));
            cout << "Sending query: " << builder.to_string() << endl;
            http_response response = client.request(methods::GET, builder.to_string()).get();

            if (response.status_code() == 200) {
                auto json_response = response.extract_json(true).get();
                cout << json_response << endl;
                cout << "Was move successful? " << json_response[0] << endl;
            }
            else {
                cout << "Error!" << endl;
            }

            next_move = astar.getNextMove();
    }

    return 0;
}

