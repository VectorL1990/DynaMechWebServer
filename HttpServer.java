public class Server
{
    public static void main(String[] args) throws IOException
    {
        //listening port
        ServerSocket ss = new ServerSocket(8080);
        System.out.println("server is running");
        for (;;)
        {
            Socket sock = ss.accept();
            System.out.println("connected from " + sock.getRemoteSocketAddress());
            Thread t = new Handler(sock);
            t.start();
        }
    }
}

class Handler extends Thread
{
    Socket sock;

    public Handler(Socket sock)
    {
        this.sock = sock;
    }

    public void run()
    {
        try (InputStream input = this.sock.getInputStream())
        {
            try (OutputStream output = this.sock.getOutputStream())
            {
                handle(input, output);
            }
        }
        catch (Exception e)
        {
            try
            {
                this.sock.close();
            }
            catch (IOException ioe)
            {

            }
            System.out.println("client disconnected.");
        }
    }

    private void handle(InputStream input, OutputStream output) throws IOException
    {
        var reader = new BufferedReader(new InputStreamReader(input, StandardCharsets.UTF_8));
        var writer = new BufferedWriter(new OutputStreamWriter(output, StandardCharsets.UTF_8));

        boolean requestOK = false;
        String first = reader.readLine();
        if (first.startsWith("GET / HTTP/1."))
        {
            requestOK = true;
        }
        for (;;)
        {
            String header = reader.readLine();
            if (header.isEmpty())
            {
                break;
            }
            System.out.println(header);
        }
        System.out.println(requestOk ? "Response OK" : "Response Error");
        if (!requestOk) {
            writer.write("HTTP/1.0 404 Not Found\r\n");
            writer.write("Content-Length: 0\r\n");
            writer.write("\r\n");
            writer.flush();
        } else {
            String data = "<html><body><h1>Hello, world!</h1></body></html>";
            int length = data.getBytes(StandardCharsets.UTF_8).length;
            writer.write("HTTP/1.0 200 OK\r\n");
            writer.write("Connection: close\r\n");
            writer.write("Content-Type: text/html\r\n");
            writer.write("Content-Length: " + length + "\r\n");
            writer.write("\r\n");
            writer.write(data);
            writer.flush();
        }
    }
}