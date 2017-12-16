package com.example.myapplication;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;
import java.util.TreeMap;

import android.annotation.SuppressLint;
import android.app.ActionBar;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.drawable.BitmapDrawable;
import android.os.Build;
import android.os.Bundle;
import android.support.constraint.ConstraintLayout;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Menu;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;
import amr.plt.rcParkingRobot.AndroidHmiPLT;
import android.support.v7.app.AppCompatActivity;

import amr.plt.rcParkingRobot.IAndroidHmi;
import parkingRobot.IGuidance;
import parkingRobot.INavigation;
import parkingRobot.INxtHmi;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;



public class MainActivity extends AppCompatActivity {

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

    //Skalierungswerte
    static final double XSKAL = 2.51;
    static final double YSKAL = 3.00;
    static final double SKALROBO = 2.51;
    static final double BREITE_AUTO = 32;
    static final double HOEHE_AUTO = 20;

    float xBild, yBild;
    float xRobo, yRobo;
    float imageX, imageY;
    float angle;

    //relevant für die Anzeige der Parklücken (als Button)
    TreeMap<Integer, String> treeMap;
    String status;

    //für Zugriff benötigt
    String status1;
    String status2;
    String status3;
    String status4;
    String status5;
    String status6;
    String status7;
    String status8;


    Canvas canvas;
    Paint pinsel;
    ImageView imageRobo;

    @SuppressLint("NewApi")
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //Bild austauschen - Test
        ImageView wlanRot = (ImageView) findViewById(R.id.wlanBack);
        wlanRot.setImageResource(R.drawable.achtung);
        Toast.makeText(this, "ACHTUNG!! Abstand wird eng!!", Toast.LENGTH_LONG).show();

        //TreeMap initialisieren
        treeMap = new TreeMap<>();

        //Anfangswerte setzen
        xRobo = 0;
        yRobo = 0;


        imagePositionieren(xRobo, yRobo);

        //Button ParkinSlot einfügen
        /*RelativeLayout rl = (RelativeLayout) findViewById(R.id.layoutParkingSlots);
        Button btn = new Button(this);
        btn.setWidth(100);
        btn.setHeight(60);
        btn.setText("1");
        btn.setEnabled(false);
        btn.setId(R.id.button_ParkingSlot1);
        btn.setX(340);
        btn.setY(310);
        btn.setBackgroundColor(Color.GREEN);
        btn.setTextColor(Color.BLACK);
        rl.addView(btn);*/

        //Button button_PS = (Button) findViewById(R.id.button_ParkingSlot);
        //button_PS.setBackgroundColor(Color.WHITE);


        RelativeLayout layout = (RelativeLayout) findViewById(R.id.viewLayout);
        pinsel = new Paint();
        pinsel.setColor(Color.rgb(64, 64, 255));
        pinsel.setStrokeWidth(5);

        Bitmap bitmap = Bitmap.createBitmap(1024, 552, Bitmap.Config.ARGB_8888);
        canvas = new Canvas(bitmap);
        layout.setBackground(new BitmapDrawable(bitmap));

        //Anfangspunkt setzen
        zeichnen(xRobo, yRobo, canvas, pinsel);


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
                if(hmiModule.isConnected() == true) {
                    //disconnect Button ausführen
                    terminateBluetoothConnection();

                    //Button Beschriftung in Connect ändern
                    connectButton.setText("Connect");
                }else{
                    Intent serverIntent = new Intent(getApplicationContext(), BluetoothActivity.class);
                    startActivityForResult(serverIntent, REQUEST_SETUP_BT_CONNECTION);

                    //Button Beschriftung in Disconnect ändern
                    connectButton.setText("Disconnect");
                }

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

                    //Beschriftung ändern
                } else{
                    // otherwise change mode to PAUSE
                    hmiModule.setMode(INxtHmi.Mode.PAUSE);
                    Log.e("Toggle","Toggled to Pause");
                }
            }
        });

        //Button Clear -> reinigt die Image Anzeige
        final Button clearButton = (Button) findViewById(R.id.buttonClear);
        clearButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v){
               /** Code einfügen */
               //neues Canvas ertellen, altes löschen
                RelativeLayout layout = (RelativeLayout) findViewById(R.id.viewLayout);

                Bitmap bitmap = Bitmap.createBitmap(1024, 552, Bitmap.Config.ARGB_8888);
                canvas = new Canvas(bitmap);
                layout.setBackground(new BitmapDrawable(bitmap));

            }
        });




        if (treeMap.size() != 0) {
            //Bestimmen, was passiert, wenn man auf die Parkbuttons drückt
            final Button parkSlot1 = (Button) findViewById(R.id.button_ParkingSlot1);
            final int id1 = R.id.button_ParkingSlot1;
            parkSlot1.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                        if (treeMap.get(id1) != null) {
                            //
                            status1 = treeMap.get(id1);
                            if (treeMap.get(id1) == "suitable") {

                            } else if (treeMap.get(id1) == "notsuitable") {
                                Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_LONG).show();
                            } else {
                                //System.out.println("Es ist ein Fehler aufgetreten.");
                                Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                            }
                        }
                    }
            });

            final Button parkSlot2 = (Button) findViewById(R.id.button_ParkingSlot2);
            final int id2 = R.id.button_ParkingSlot2;
            parkSlot2.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                        if (treeMap.get(id2) != null) {
                            //
                            status2 = treeMap.get(id2);
                            if (treeMap.get(id2) == "suitable") {

                            } else if (treeMap.get(id2) == "notsuitable") {
                                Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_LONG).show();
                            } else {
                                //System.out.println("Es ist ein Fehler aufgetreten.");
                                Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                            }
                        }
                    }
            });

            final Button parkSlot3 = (Button) findViewById(R.id.button_ParkingSlot3);
            final int id3 = R.id.button_ParkingSlot3;
            parkSlot3.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                        if (treeMap.get(id3) != null) {
                            //
                            status3 = treeMap.get(id3);
                            if (treeMap.get(id3) == "suitable") {

                            } else if (treeMap.get(id3) == "notsuitable") {
                                Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_LONG).show();
                            } else {
                                //System.out.println("Es ist ein Fehler aufgetreten.");
                                Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                            }
                        }
                    }
            });

            final Button parkSlot4 = (Button) findViewById(R.id.button_ParkingSlot4);
            final int id4 = R.id.button_ParkingSlot4;
            parkSlot4.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                        if (treeMap.get(id4) != null) {
                            //
                            status4 = treeMap.get(id4);
                            if (treeMap.get(id4) == "suitable") {

                            } else if (treeMap.get(id4) == "notsuitable") {
                                Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_LONG).show();
                            } else {
                                //System.out.println("Es ist ein Fehler aufgetreten.");
                                Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                            }
                        }
                    }
            });

            final Button parkSlot5 = (Button) findViewById(R.id.button_ParkingSlot5);
            final int id5 = R.id.button_ParkingSlot5;
            parkSlot5.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                        if (treeMap.get(id5) != null) {
                            //
                            status5 = treeMap.get(id5);
                            if (treeMap.get(id5) == "suitable") {

                            } else if (treeMap.get(id5) == "notsuitable") {
                                Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_LONG).show();
                            } else {
                                //System.out.println("Es ist ein Fehler aufgetreten.");
                                Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                            }
                        }
                    }
            });

            final Button parkSlot6 = (Button) findViewById(R.id.button_ParkingSlot6);
            final int id6 = R.id.button_ParkingSlot6;
            parkSlot6.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                        if (treeMap.get(id6) != null) {
                            //
                            status6 = treeMap.get(id6);
                            if (treeMap.get(id6) == "suitable") {

                            } else if (treeMap.get(id6) == "notsuitable") {
                                Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_LONG).show();
                            } else {
                                //System.out.println("Es ist ein Fehler aufgetreten.");
                                Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                            }
                        }
                    }
            });

            final Button parkSlot7 = (Button) findViewById(R.id.button_ParkingSlot7);
            final int id7 = R.id.button_ParkingSlot7;
            parkSlot7.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                        if (treeMap.get(id7) != null) {
                            //
                            status7 = treeMap.get(id7);
                            if (treeMap.get(id7) == "suitable") {

                            } else if (treeMap.get(id7) == "notsuitable") {
                                Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_LONG).show();
                            } else {
                                //System.out.println("Es ist ein Fehler aufgetreten.");
                                Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                            }
                        }
                    }
            });

            final Button parkSlot8 = (Button) findViewById(R.id.button_ParkingSlot8);
            final int id8 = R.id.button_ParkingSlot8;
            parkSlot8.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                        if (treeMap.get(id8) != null) {
                            //
                            status8 = treeMap.get(id8);
                            if (treeMap.get(id8) == "suitable") {

                            } else if (treeMap.get(id8) == "notsuitable") {
                                Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_LONG).show();
                            } else {
                                //System.out.println("Es ist ein Fehler aufgetreten.");
                                Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                            }
                        }
                    }
            });
        }
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


                    displayDataNXT(); //method also exists in the class DataAvtivity.java

                    //aktuelle Werte für die Value-Anzeigen in der MainActivity setzen
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
                            //rudimentäres Zeichnen des gefahrenen Pfades
                            xRobo = hmiModule.getPosition().getX();
                            yRobo = hmiModule.getPosition().getY();
                            zeichnen(xRobo, yRobo, canvas, pinsel);
                            imagePositionieren(xRobo, yRobo);


                            //display bluetooth connection status
                            final TextView modeValue = (TextView) findViewById(R.id.textViewModeValue);
                            modeValue.setText(String.valueOf(hmiModule.getCurrentStatus()));


                            //restart activity when disconnecting
                            if (hmiModule.getCurrentStatus() == IGuidance.CurrentStatus.EXIT) {
                                terminateBluetoothConnection();
                                restartActivity();
                            }

                            //Parklückenanzeige
                            //Methode wird nur aufgerufen, wenn es mindestens eine Parklücke gibt
                            //parkSlots();
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

     private void zeichnen(float xRobo, float yRobo, Canvas canvas, Paint pinsel){

         xBild = (float) ((xRobo * XSKAL) + 100);
         yBild = (float) (((yRobo * YSKAL) - 295) * (-1));

         //Elemente (Punkte) zeichnen lassen
         canvas.drawCircle(xBild, yBild, 5, pinsel);

         canvas.drawLine(0, 0, 1024, 552, pinsel);

     }

     private void imagePositionieren(float xRobo, float yRobo){
         //image verschieben
         imageRobo = (ImageView) findViewById(R.id.robo);

         //Winkel des Bildes getreu des aktuellen Winkels des Roboters setzen
         if(hmiModule == null){
             angle = 0;
         }else{
             angle = hmiModule.getPosition().getAngle();
         }


         //Werte umrechnen
         imageX = (float) ((xRobo * SKALROBO) + (100 - BREITE_AUTO));
         imageY = (float) (((yRobo * SKALROBO) - (248 - HOEHE_AUTO)) * (-1));

         //Image verschieben (positionieren)
         imageRobo.setY(imageY);
         imageRobo.setX(imageX);
         imageRobo.setRotation(angle);


     }

     private void parkSlots() {
         int number = hmiModule.getNoOfParkingSlots();
         AndroidHmiPLT.ParkingSlot parkingSlot;
         float breite, hoehe;
         float xSet, ySet;
         float xPos, yPos;

         //int-Werte der IDs setzen
         ArrayList<Integer> ids = new ArrayList<>();
         ids.add(0, R.id.button_ParkingSlot1);
         ids.add(1, R.id.button_ParkingSlot2);
         ids.add(2, R.id.button_ParkingSlot3);
         ids.add(3, R.id.button_ParkingSlot4);
         ids.add(4, R.id.button_ParkingSlot5);
         ids.add(5, R.id.button_ParkingSlot6);
         ids.add(6, R.id.button_ParkingSlot7);
         ids.add(7, R.id.button_ParkingSlot8);



         //nur nach Parklücken-Details suchen, wenn auch welche detektiert wurden
         if (number != 0) {
             for (int i = 0; i < number; i++) {
                 parkingSlot = hmiModule.getParkingSlot(i);

                     //Position auswählen
                     //inklusive Reservepuffer
                     if (parkingSlot.getBackBoundaryPosition().x <= 175) {
                         //Button rechts im Parcours einfügen
                         hoehe = Math.abs(parkingSlot.getBackBoundaryPosition().y - parkingSlot.getFrontBoundaryPosition().y);
                         hoehe = (float) (((hoehe * YSKAL) - 273) * (-1));

                         //Button ParkinSlot einfügen
                         RelativeLayout rl = (RelativeLayout) findViewById(R.id.layoutParkingSlots);
                         Button btn = new Button(this);
                         btn.setWidth(60);
                         btn.setHeight((int) hoehe);
                         btn.setText(String.valueOf(i+1));
                         btn.setEnabled(false);
                         btn.setId(ids.get(i));

                         //Werte umrechnen
                         xPos = parkingSlot.getBackBoundaryPosition().x;
                         yPos = parkingSlot.getBackBoundaryPosition().y;
                         xSet = (float) ((xPos * XSKAL) + 306);
                         ySet = (float) (((yPos * YSKAL) - 325) * (-1));

                         btn.setX(xSet);
                         btn.setY(ySet);
                         btn.setTextColor(Color.BLACK);

                         //Farbe setzen
                         if(parkingSlot.getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING) {
                             btn.setBackgroundColor(Color.GREEN);
                             status = "suitable";
                         }else {
                             btn.setBackgroundColor(Color.RED);
                             status = "notsuitable";
                         }

                         rl.addView(btn);

                         //Wert der TreeMap hinzufügen
                         treeMap.put(ids.get(i), status);

                     } else if (parkingSlot.getBackBoundaryPosition().y < 20) {     //Offset berücksichtigen
                         //Button unterhalb des Roboter einfügen
                         breite = Math.abs(parkingSlot.getBackBoundaryPosition().x - parkingSlot.getFrontBoundaryPosition().x);
                         breite = (float) ((breite * XSKAL) + 306);

                         //Button ParkinSlot einfügen
                         RelativeLayout rl = (RelativeLayout) findViewById(R.id.layoutParkingSlots);
                         Button btn = new Button(this);
                         btn.setWidth((int) breite);
                         btn.setHeight(60);
                         btn.setText(String.valueOf(i+1));
                         btn.setEnabled(false);
                         btn.setId(ids.get(i));

                         //Werte umrechnen
                         xPos = parkingSlot.getBackBoundaryPosition().x;
                         yPos = parkingSlot.getBackBoundaryPosition().y;
                         xSet = (float) ((xPos * XSKAL) + 306);
                         ySet = (float) (((yPos * YSKAL) - 325) * (-1));

                         btn.setX(xSet);
                         btn.setY(ySet);
                         btn.setTextColor(Color.BLACK);

                         //Farbe setzen
                         if(parkingSlot.getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING) {
                             btn.setBackgroundColor(Color.GREEN);
                             status = "suitable";
                         }else {
                             btn.setBackgroundColor(Color.RED);
                             status = "notsuitable";
                         }

                         rl.addView(btn);

                         //TreeMap setzen
                         treeMap.put(ids.get(i), status);

                     } else if (parkingSlot.getBackBoundaryPosition().y >= 20) {    //Offset berücksichtigen
                         //Button unterhalb des Roboter einfügen
                         breite = Math.abs(parkingSlot.getBackBoundaryPosition().x - parkingSlot.getFrontBoundaryPosition().x);
                         breite = (float) ((breite * XSKAL) + 306);

                         //Button ParkinSlot einfügen
                         RelativeLayout rl = (RelativeLayout) findViewById(R.id.layoutParkingSlots);
                         Button btn = new Button(this);
                         btn.setWidth((int) breite);
                         btn.setHeight(60);
                         btn.setText(String.valueOf(i+1));
                         btn.setEnabled(false);
                         btn.setId(ids.get(i));

                         //Werte umrechnen
                         xPos = parkingSlot.getBackBoundaryPosition().x;
                         yPos = parkingSlot.getBackBoundaryPosition().y;
                         xSet = (float) ((xPos * XSKAL) + 306);
                         ySet = (float) (((yPos * YSKAL) - 325) * (-1));
                         //ySet nach oben verlagern, da immer die obere linke Ecke betrachtet wird
                         ySet = ySet - 60;

                         btn.setX(xSet);
                         btn.setY(ySet);
                         btn.setTextColor(Color.BLACK);

                         //Farbe setzen
                         if(parkingSlot.getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING) {
                             btn.setBackgroundColor(Color.GREEN);
                             status = "suitable";
                         }else {
                             btn.setBackgroundColor(Color.RED);
                             status = "notsuitable";
                         }

                         rl.addView(btn);

                         //TreeMap den Wert übergeben
                         treeMap.put(ids.get(i), status);

                     }
             }
         }
     }

}
