/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.team3171.sensors;

// Java Imports
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * @author Mark Ebert
 */
public class UDPClient implements Runnable {

    private static final ScheduledExecutorService executor = Executors.newScheduledThreadPool(2);
    private Future<?> future;
    private final ConcurrentLinkedQueue<String> MESSAGE_BUFFER;
    private final DatagramSocket OUTGOING_SOCKET;
    private final InetAddress IP_ADDRESS;
    private final int PORT;

    public UDPClient(ConcurrentLinkedQueue<String> MESSAGE_BUFFER, String IP_ADDRESS, int PORT)
            throws UnknownHostException, IOException {
        this.MESSAGE_BUFFER = MESSAGE_BUFFER;
        this.IP_ADDRESS = InetAddress.getByName(IP_ADDRESS);
        this.PORT = PORT;
        this.OUTGOING_SOCKET = new DatagramSocket();
    }

    @Override
    public void run() {
        try {
            // Retrieve the data string, if there isn't one it returns null
            final String message = MESSAGE_BUFFER.poll();
            if (message != null) {
                // Take the data string and convert it to a byte array
                final byte[] outboundBytes = message.getBytes();
                // Create a new packet to send the polled data
                final DatagramPacket packet = new DatagramPacket(outboundBytes, outboundBytes.length, IP_ADDRESS, PORT);
                // Send the packet
                OUTGOING_SOCKET.send(packet);
            }
        } catch (IOException e) {
            System.err.println("Could not send any data to the Server!");
            System.err.println("Shutting down the client...");
            close();
            System.err.println("Client shutdown sucessfully!");
        }
    }

    public void start(final int updateRate) {
        if (MESSAGE_BUFFER != null && future == null) {
            future = executor.scheduleAtFixedRate(this, 0, updateRate, TimeUnit.MILLISECONDS);
        }
    }

    public void start() {
        start(10);
    }

    public void close() {
        try {
            OUTGOING_SOCKET.close();
            future.cancel(true);
            future.wait(1000);
        } catch (InterruptedException ex) {
            System.err.println(ex.getMessage());
        }
    }

    public boolean isAlive() {
        return !(future.isCancelled() || OUTGOING_SOCKET.isClosed());
    }

}
