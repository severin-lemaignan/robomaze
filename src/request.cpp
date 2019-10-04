#include <iostream>
#include <string>

#include <cpprest/http_client.h>

using namespace std;
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features

int main(int argc, char* argv[])
{

    if (argc < 3) {
        cout << "Usage: request <name robot> <direction>" << endl;
        return 1;
    }

    // Create http_client to send the request.
    http_client client(U("https://robomaze.skadge.org/"));

    // Build request URI and start the request.
    uri_builder builder(U("/api"));
    builder.append_query(U("move"), U("[\"" + string(argv[1]) +"\",\"" + string(argv[2]) + "\"]"));
    cout << "Sending query: " << builder.to_string() << endl;
    http_response response = client.request(methods::GET, builder.to_string()).get();

    cout << "Received response status code:" << response.status_code() << endl;

    //cout << response.extract_string(true).get() << endl;

    // extract the JSON response
    if (response.status_code() == 200) {
        auto json_response = response.extract_json(true).get();
        cout << json_response << endl;
        cout << "Was move successful? " << json_response[0] << endl;
    }
    else {
        cout << "Error!" << endl;
    }

    return 0;
}

