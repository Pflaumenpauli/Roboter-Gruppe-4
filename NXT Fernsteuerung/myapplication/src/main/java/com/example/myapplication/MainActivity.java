package com.example.myapplication;

import java.util.Timer;
import java.util.TimerTask;

import android.app.ActionBar;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.graphics.Canvas;
import android.os.Build;
import android.os.Bundle;
import android.support.constraint.ConstraintLayout;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Menu;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;
import amr.plt.rcParkingRobot.AndroidHmiPLT;
import android.support.v7.app.AppCompatActivity;

import parkingRobot.IGuidance;
import parkingRobot.INxtHmi;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;



public class MainActivity extends AppCompatActivity {
    private ZeichenView zview;

    //representing local Bluetooth adapter
    BluetoothAdapter mBtAdapter = null;
    //representing the bluetooth hardware device
    BluetoothDevice btDevice = null;
    //instance handels bluetooth communication to NXT

    public static AndroidHmiPLT hmiModule = null;
    //request code
    final int REQUEST_SETUP_BT_CONNECTION = 1;

    //name and address of the device
    String btDeviceAddress="";
    String btDeviceName="";


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        zview =new ZeichenView(this);
        zview.setLayoutParams(new ActionBar.LayoutParams(ActionBar.LayoutParams.MATCH_PARENT, ActionBar.LayoutParams.MATCH_PARENT));
        zview.setBackgroundColor(Color.TRANSPARENT);

        ConstraintLayout layout = (ConstraintLayout) findViewById(R.id.constraintLayout);
        layout.addView(zview);

       /* Context context = getApplicationContext();
        DisplayMetrics displayMetrics = context.getResources().getDisplayMetrics();
        float dpHeight = displayMetrics.heightPixels / displayMetrics.density;
        float dpWidth = displayMetrics.widthPixels / displayMetrics.density;
        System.out.println("dpHeight = " + dpHeight);
        System.out.println("dpWidth = " + dpWidth);*/


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
                /** Code einfügen */

                //set button Scout or button park_this or ausparken as unchecked
                final ToggleButton toggleButton = (ToggleButton) findViewById(R.id.toggleMode);
                final Button park_this = (Button) findViewById(R.id.buttonParkThis);
                final Button ausparken = (Button) findViewById(R.id.buttonAusparken);

                //wir gehen davon aus, dass nach Pause wieder in den Scout Modus zurück gekehrt wird
                toggleButton.setEnabled(true);
                toggleButton.setChecked(false);

                if(park_this.isEnabled() == true){
                    park_this.setEnabled(false);
                }
                if(ausparken.isEnabled() == true){
                    ausparken.setEnabled(false);
                }

                hmiModule.setMode(INxtHmi.Mode.PAUSE);
                Log.e("Toggle","Toggled to Pause");
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

                    displayDataNXT(); //method also exists in the class DataAvtivity.java

                    //aktuelle Werte für die Value-Anzeigen in der MainActivity setzen
                    TextView xValue = (TextView) findViewById(R.id.textViewXPositionValue);
                    xValue.setText(String.valueOf(hmiModule.getPosition().getX() + " cm"));

                    TextView yValue = (TextView) findViewById(R.id.textViewYPositionValue);
                    yValue.setText(String.valueOf(hmiModule.getPosition().getY() + " cm"));

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
        btDeviceAddress = btDevice.getAddress();
        btDeviceName = btDevice.getName();

        //instantiate client modul
        hmiModule = new AndroidHmiPLT(btDeviceName, btDeviceAddress);

        //connect to the specified device
        hmiModule.connect();

        //wait till connection really is established and
        int i = 0;
        while (!hmiModule.isConnected()&& i<100000000/2) {
            i++;
        }
    }

    /**
     * Display the current data of NXT
     */
    private void displayDataNXT() {

        new Timer().schedule(new TimerTask() {

            @Override
            public void run() {

                runOnUiThread(new Runnable() {
                    public void run() {
                        if (hmiModule != null) {

                            //display bluetooth connection status
                            final TextView modeValue = (TextView) findViewById(R.id.textViewModeValue);
                            modeValue.setText(String.valueOf(hmiModule.getCurrentStatus()));

                            //display the values of the current position
                            final TextView xValue = (TextView) findViewById(R.id.textViewXPositionValue);
                            xValue.setText(String.valueOf(hmiModule.getPosition().getX() + " cm"));
                            final TextView yValue = (TextView) findViewById(R.id.textViewYPositionValue);
                            yValue.setText(String.valueOf(hmiModule.getPosition().getY() + " cm"));

                            //restart activity when disconnecting
                            if (hmiModule.getCurrentStatus() == IGuidance.CurrentStatus.EXIT) {
                                terminateBluetoothConnection();
                                restartActivity();
                            }
                        }
                    }
                });
            }
        }, 200, 100);
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

        //start a new MainActivity
        Intent serverIntent = new Intent(getApplicationContext(),MainActivity.class);
        startActivityForResult(serverIntent, REQUEST_SETUP_BT_CONNECTION);

        //Alternative zum Code mit neuem Intent
        //Oberfläche zurücksetzen
        //enable bluetooth connection button
        /*final Button connectButton = (Button) findViewById(R.id.buttonSetupBluetooth);
        connectButton.setEnabled(true);
        //disable scout, Park_this, ausparken and disconnect button
        final ToggleButton toggleMode = (ToggleButton) findViewById(R.id.toggleMode);
        toggleMode.setEnabled(false);
        final Button parkThis = (Button) findViewById(R.id.buttonParkThis);
        parkThis.setEnabled(false);
        final Button ausparken = (Button) findViewById(R.id.buttonAusparken);
        ausparken.setEnabled(false);
        final Button disconnectButton = (Button) findViewById(R.id.buttonDisconnect);
        disconnectButton.setEnabled(false);*/

    }

    /**
     * restart the activity
     */
    private void restartActivity(){
        Intent restartIntent = new Intent(getApplicationContext(),MainActivity.class);
        startActivity(restartIntent);
        finish();
    }

    public static AndroidHmiPLT getHmiModule(){
        return hmiModule;
    }




     /*public boolean onTouchEvent(MotionEvent e) {
        float xpos= e.getX();
        float ypos= e.getRawY();
        switch (e.getAction())
        {
            case MotionEvent.ACTION_DOWN:
                Log.d("DEBUG", "On touch (down)" + String.valueOf(xpos) + String.valueOf(ypos));
            case MotionEvent.ACTION_UP:
                Log.d("DEBUG", "On touch (up)" + String.valueOf(xpos) + String.valueOf(ypos));
            case MotionEvent.ACTION_MOVE:
                Log.d("DEBUG", "On touch (move)" + String.valueOf(xpos) + String.valueOf(ypos));
                break;
        }
        System.out.println("xPos: " + xpos);
        System.out.println("yPos: " + ypos);

        return true;

    }*/

}
