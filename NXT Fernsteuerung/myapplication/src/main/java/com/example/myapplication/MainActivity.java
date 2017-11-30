package com.example.myapplication;

import java.util.Timer;
import java.util.TimerTask;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;
import amr.plt.rcParkingRobot.AndroidHmiPLT;
import android.support.v7.app.AppCompatActivity;
import parkingRobot.IGuidance;
import parkingRobot.INxtHmi;
import amr.plt.rcParkingRobot.BTCommunicationThread;


public class MainActivity extends AppCompatActivity {

    //representing local Bluetooth adapter
    BluetoothAdapter mBtAdapter = null;
    //representing the bluetooth hardware device
    BluetoothDevice btDevice = null;
    //instance handels bluetooth communication to NXT

    AndroidHmiPLT hmiModule = null;
    //request code
    final int REQUEST_SETUP_BT_CONNECTION = 1;

    //dafür wurde die Klasse BTCommunicationThread nachträglich als public deklariert
    BTCommunicationThread btCommThread = null;


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //get the BT-Adapter
        mBtAdapter = BluetoothAdapter.getDefaultAdapter();

        //If the adapter is null, then Bluetooth is not supported
        if (mBtAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        final Button connectButton = (Button) findViewById(R.id.buttonSetupBluetooth);
        //on click call the BluetoothActivity to choose a listed device
        connectButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v){
                Intent serverIntent = new Intent(getApplicationContext(),BluetoothActivity.class);
                startActivityForResult(serverIntent, REQUEST_SETUP_BT_CONNECTION);
            }
        });

        //toggle button allows user to set mode of the NXT device
        final ToggleButton toggleButton = (ToggleButton) findViewById(R.id.toggleMode);
        //disable button initially
        toggleButton.setEnabled(false);
        //on click change mode
        toggleButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v){
                boolean checked = ((ToggleButton) v).isChecked();
                if (checked) {
                    //if toggle is checked change mode to SCOUT
                    hmiModule.setMode(INxtHmi.Mode.SCOUT);
                    Log.e("Toggle","Toggled to Scout");
                } else{
                    // otherwise change mode to PAUSE
                    hmiModule.setMode(INxtHmi.Mode.PAUSE);
                    Log.e("Toggle","Toggled to Pause");
                }
            }
        });


        //ToggleButton Pause
        final Button buttonPause = (Button) findViewById(R.id.buttonPause);
        buttonPause.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v){
                //Pause
                hmiModule.setMode(INxtHmi.Mode.PAUSE);
                /** Code einfügen */
            }
        });


        //what happen when someone press the data button?
        final Button dataButton = (Button) findViewById(R.id.buttonShowData);
        //on click call the DataActivity to see the data from the sensors
        dataButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v){
                Intent serverIntent = new Intent(getApplicationContext(),DataActivity.class);
                startActivityForResult(serverIntent, REQUEST_SETUP_BT_CONNECTION);
            }
        });


        //someone press the Park_This button
        final Button parkThisButton = (Button) findViewById(R.id.buttonParkThis);
        //on click call the DataActivity to see the data from the sensors
        parkThisButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v){
                hmiModule.setMode(INxtHmi.Mode.PARK_THIS);
                /** Code einfügen */
            }
        });

        //someone press the Ausparken button
        //soll nur nach dem Einparken ansprechbar sein
        final Button buttonAusparken = (Button) findViewById(R.id.buttonAusparken);
        buttonAusparken.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v){

                /** Code einfügen */
            }
        });


        //someone press the disconnect button
        final Button disconnectButton = (Button) findViewById(R.id.buttonDisconnect);
        //on click call the DataActivity to see the data from the sensors
        disconnectButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v){

                terminateBluetoothConnection();
            }
        });

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        super.onCreateOptionsMenu(menu);
        return true;
    }

    @Override
    public void onDestroy(){
        super.onDestroy();
        if(mBtAdapter != null){
            //release resources
            mBtAdapter.cancelDiscovery();
        }

    }

    /**
     * handle pressing button with alert dialog if connected(non-Javadoc)
     * @see android.app.Activity#onBackPressed()
     */
    @Override
    public void onBackPressed() {
        super.onBackPressed();

        if (hmiModule != null && hmiModule.connected) {
            //creating new AlertDialog
            AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setMessage("Are you sure you want to terminate the connection?")
                    .setCancelable(false)
                    .setPositiveButton("Yes", new DialogInterface.OnClickListener() {
                        public void onClick(DialogInterface dialog, int id) {
                            //disconnect and return to initial screen
                            terminateBluetoothConnection();
                            restartActivity();
                        }
                    })
                    .setNegativeButton("No", new DialogInterface.OnClickListener() {
                        public void onClick(DialogInterface dialog, int id) {
                            dialog.cancel();
                        }
                    });
            AlertDialog alert = builder.create();
            alert.show();
        }
    }

    /**
     * instantiating AndroidHmiPlt object and display received data(non-Javadoc)
     * @see android.app.Activity#onActivityResult(int, int, android.content.Intent)
     */
    public void onActivityResult(int requestCode, int resultCode, Intent data){
        switch(resultCode){

            //user pressed back button on bluetooth activity, so return to initial screen
            case Activity.RESULT_CANCELED:
                break;
            //user chose device
            case Activity.RESULT_OK:
                //connect to chosen NXT
                establishBluetoothConnection(data);
                //display received data from NXT
                if(hmiModule.connected){
                    //After establishing the connection make sure the start mode of the NXT is set to PAUSE
                    //hmiModule.setMode(Mode.PAUSE);

                    //enable toggle button (Scout)
                    final ToggleButton toggleMode = (ToggleButton) findViewById(R.id.toggleMode);
                    toggleMode.setEnabled(true);

                    //enable Park_This button
                    final Button toggleParkThis = (Button) findViewById(R.id.buttonParkThis);
                    toggleParkThis.setEnabled(true);

                    //enable disconnect button
                    final Button disconnectButton = (Button) findViewById(R.id.buttonDisconnect);
                    disconnectButton.setEnabled(true);

                    //disable connect button
                    final Button connectButton = (Button) findViewById(R.id.buttonSetupBluetooth);
                    connectButton.setEnabled(false);

                   // displayDataNXT();
                    //diese Methode wurde in die Klasse DataActivity verschoben

                    //aktuelle Werte für die Value-Anzeigen in der MainActivity setzen
                    TextView xValue = (TextView) findViewById(R.id.textViewXPositionValue);
                    xValue.setText(String.valueOf(hmiModule.getPosition().getX()));

                    TextView yValue = (TextView) findViewById(R.id.textViewYPositionValue);
                    yValue.setText(String.valueOf(hmiModule.getPosition().getY()));

                    TextView mode = (TextView) findViewById(R.id.textViewModeValue);
                    mode.setText(String.valueOf(hmiModule.getCurrentStatus()));
                    break;
                } else{
                    Toast.makeText(this, "Bluetooth connection failed!", Toast.LENGTH_SHORT).show();
                    Toast.makeText(this, "Is the selected NXT really present & switched on?", Toast.LENGTH_LONG).show();
                    break;
                }
        }
    }

    /**
     * Connect to the chosen device
     * @param data
     */
    private void establishBluetoothConnection(Intent data){
        //get instance of the chosen bluetooth device
        String address = data.getExtras().getString(BluetoothActivity.EXTRA_DEVICE_ADDRESS);
        btDevice = mBtAdapter.getRemoteDevice(address);

        //get name and address of the device
        String btDeviceAddress = btDevice.getAddress();
        String btDeviceName = btDevice.getName();

        //instantiate client modul
        hmiModule = new AndroidHmiPLT(btDeviceName, btDeviceAddress);
        //daraufhin kann BTCommunicationThread initialisiert werden
        //dafür wurde der Konstruktor nachträglich als public deklariert
        btCommThread = new BTCommunicationThread(hmiModule);


        //connect to the specified device
        hmiModule.connect();

        //wait till connection really is established and
        int i = 0;
        while (!hmiModule.isConnected()&& i<100000000/2) {
            i++;
        }

        if(hmiModule.isConnected() == true){
            btCommThread.run();
        }else{
            System.out.println("Not connected to any NXT.");
        }
    }


    /**
     * Terminate the bluetooth connection to NXT
     */
    private void terminateBluetoothConnection(){
        Toast.makeText(this, "Bluetooth connection was terminated!", Toast.LENGTH_LONG).show();
        hmiModule.setMode(INxtHmi.Mode.DISCONNECT);
        hmiModule.disconnect();

        while(hmiModule.isConnected()){
            //wait until disconnected
        }
        hmiModule = null;
    }

    /**
     * restart the activity
     */
    private void restartActivity(){
        Intent restartIntent = new Intent(getApplicationContext(),MainActivity.class);
        startActivity(restartIntent);
        finish();
    }

}
