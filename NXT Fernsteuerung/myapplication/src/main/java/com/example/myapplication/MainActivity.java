package com.example.myapplication;

import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;
import java.util.TreeMap;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.DialogInterface;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.PointF;
import android.graphics.drawable.BitmapDrawable;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;
import amr.plt.rcParkingRobot.AndroidHmiPLT;
import android.support.v7.app.AppCompatActivity;

import amr.plt.rcParkingRobot.IAndroidHmi;
import parkingRobot.IGuidance;
import parkingRobot.INxtHmi;


import android.graphics.Color;
import android.graphics.Paint;


/**
 * MainActivity for the user to control and navigate the robot. In this Activity is shown the static map, the driven distance and the mode value.
 * With the aid of buttons the user can regulate the mode of the robot and can choose the parking slot where the robot has to park in autonomous.
 */
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

    //scaling values
    static final double XSKAL = 2.51;
    static final double YSKAL = 3.00;
    static final double SKALROBO = 2.51;
    static final double BREITE_AUTO = 32;
    static final double HOEHE_AUTO = 20;

    // needed attributes to show the driven path
    float xBild, yBild;
    float xRobo, yRobo;
    float imageX, imageY;
    float angle;

    // needed to calculate the distance of the driven path
    float xNeu = 0, xAlt = 0, yNeu = 0, yAlt = 0;
    float strecke = 0;
    String streckeAnzeige;
    static final float ABWEICHUNG = 0;       //which difference is allowed
    ArrayList<Float> arrayXRobo = new ArrayList<>();
    ArrayList<Float> arrayYRobo = new ArrayList<>();

    //to show the right images for the sensors in front/ back of the car -> true, if the attention picture is shown
    boolean frontAttention = false;
    boolean backAttention = false;

    // two ArrayLists to delete the path after some time
    ArrayList<Float> arrayX = new ArrayList<>();
    ArrayList<Float> arrayY = new ArrayList<>();

    // important to show the parking slots (as buttons)
    TreeMap<Integer, String> treeMap;
    String status;
    boolean oneNotSuitable = false;
    int numberOld, numberNew, number;
    static final int VERSCHIEBUNG = 40;     //correction value based on the path in the map -> movement for the parking slots


    Canvas canvas;
    Paint pinsel;
    ImageView imageRobo;


    /**
     * This method is called at the beginning of the app (when it is started the first time).
     * @param savedInstanceState
     */
    @SuppressLint("NewApi")
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //initialize the value of the distance
        strecke= 0;

        //initialize TreeMap
        treeMap = new TreeMap<>();

        //set the initial value
        xRobo = 0;
        yRobo = 0;
        imagePositionieren(xRobo, yRobo);

        numberOld = 0;
        numberNew = 0;


        RelativeLayout layout = (RelativeLayout) findViewById(R.id.viewLayout);
        pinsel = new Paint();
        pinsel.setColor(Color.BLUE);
        //pinsel.setColor(Color.rgb(64, 64, 255));
        pinsel.setStrokeWidth(5);

        //set initial values -> create at first a new canvas
        canvas = getNewCanvas();
        //zeichnen(xRobo, yRobo, canvas, pinsel);

        testenPS();


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
                if(hmiModule == null){
                    //erster Fall muss separat ausgeführt werden
                    Intent serverIntent = new Intent(getApplicationContext(), BluetoothActivity.class);
                    startActivityForResult(serverIntent, REQUEST_SETUP_BT_CONNECTION);

                }else if(hmiModule.isConnected() == true) {
                    //disconnect Button ausführen
                    terminateBluetoothConnection();

                    //label of the buttons have to change automatcally at the start of a new MainActivity
                }else{
                    Intent serverIntent = new Intent(getApplicationContext(), BluetoothActivity.class);
                    startActivityForResult(serverIntent, REQUEST_SETUP_BT_CONNECTION);

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
                } else{
                    // otherwise change mode to PAUSE
                    hmiModule.setMode(INxtHmi.Mode.PAUSE);
                    Log.e("Toggle","Toggled to Pause");
                }
            }
        });

        //Button Clear -> clear the image, delete the driven path in the map
        final Button clearButton = (Button) findViewById(R.id.buttonClear);
        clearButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v){
               //create a new canvas, delete the old one
                RelativeLayout layout = (RelativeLayout) findViewById(R.id.viewLayout);

                Bitmap bitmap = Bitmap.createBitmap(1024, 552, Bitmap.Config.ARGB_8888);
                canvas = new Canvas(bitmap);
                layout.setBackground(new BitmapDrawable(bitmap));

                // the arrays with the old values have to be deleted
                arrayX.clear();
                arrayY.clear();
            }
        });

        //Button TESTEN -> to test the display of the parking slots
        final ToggleButton testButton = (ToggleButton) findViewById(R.id.buttonTesten);
        testButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v){
                boolean checked = ((ToggleButton) v).isChecked();
                if(checked){
                    Button ps_1 = (Button) findViewById(R.id.PS_statisch_1);
                    Button ps_2 = (Button) findViewById(R.id.PS_statisch_2);
                    Button ps_3 = (Button) findViewById(R.id.PS_statisch_3);
                    Button ps_4 = (Button) findViewById(R.id.PS_statisch_4);

                    ps_1.setVisibility(View.VISIBLE);
                    ps_2.setVisibility(View.VISIBLE);
                    ps_3.setVisibility(View.VISIBLE);
                    ps_4.setVisibility(View.VISIBLE);

               }else {
                    Button ps_1 = (Button) findViewById(R.id.PS_statisch_1);
                    Button ps_2 = (Button) findViewById(R.id.PS_statisch_2);
                    Button ps_3 = (Button) findViewById(R.id.PS_statisch_3);
                    Button ps_4 = (Button) findViewById(R.id.PS_statisch_4);

                    ps_1.setVisibility(View.INVISIBLE);
                    ps_2.setVisibility(View.INVISIBLE);
                    ps_3.setVisibility(View.INVISIBLE);
                    ps_4.setVisibility(View.INVISIBLE);

               }

            }
        });


       /* if(oneNotSuitable == true){
            //Button Parklücke not suitable -> reinigt die Image Anzeige
            final Button notSuitableButton = (Button) findViewById(R.id.notSuitable);
            notSuitableButton.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v){
                    Toast.makeText(MainActivity.this, "Diese Parklücke ist nicht geeignet.", Toast.LENGTH_SHORT).show();
                }
            });

        }*/

        //to test the display of the ("detected") parking slots
        //testenPS();

    }


    /**
     * to specify the options menu for an activity
     * In this method, you can inflate your menu resource (defined in XML) into the Menu provided in the callback.
     * @param menu
     * @return
     */
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        super.onCreateOptionsMenu(menu);
        return true;
    }

    /**
     * This method is called if the activity is finishing or being destroyed by the system.
     */
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
                    hmiModule.setMode(INxtHmi.Mode.PAUSE);

                    //enable toggle button (Scout)
                    final ToggleButton toggleMode = (ToggleButton) findViewById(R.id.toggleMode);
                    toggleMode.setEnabled(true);

                    //change the label of the connectButton
                    final Button connectButton = (Button) findViewById(R.id.buttonSetupBluetooth);
                    connectButton.setText("DISCONNECT");


                    displayDataNXT(); //method also exists in the class DataAvtivity.java

                    //set current values to show these in the activity
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
                            //rudimentary drawing (display) of the driven path
                            xRobo = hmiModule.getPosition().getX();
                            yRobo = hmiModule.getPosition().getY();
                           // canvas = getNewCanvas();
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

                            //display detected parking slots
                            // method is called if the Navigation detected at least one parking slot
                            parkSlots();

                            //update the images of the sensors in front/ back of the car
                            sensorBilder();

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


    /**
     * This method is used to draw the driven path of the robot in the static map on the tablet display.
     * @param xRobo
     * @param yRobo
     * @param canvas
     * @param pinsel
     */
     private void zeichnen(float xRobo, float yRobo, Canvas canvas, Paint pinsel){
         xBild = (float) ((xRobo * XSKAL) + 100);
         yBild = (float) (((yRobo * YSKAL) - 295) * (-1));

         //Elemente (Punkte) zeichnen lassen
         //canvas.drawCircle(xBild, yBild, 5, pinsel);
         //canvas.drawLine(0, 0, 1024, 552, pinsel);

         //add the values to the ArrayList
         //limit the number of coordinates to 200 (which are saved and showed)

         // add the current coordinates of the robot to the ArrayLists -> limit the ArrayLists
         if (arrayXRobo.size() < 200) {
             //add the new value to the end of the list
             if(arrayXRobo.contains(xRobo) == false) {
                 arrayXRobo.add(xRobo);
                 arrayYRobo.add(yRobo);
             }
         } else {
             // delete the first value of the ArrayList (it's the oldest one) -> then add the new value at the end of the list
             if(arrayXRobo.contains(xRobo) == false) {
                 arrayXRobo.remove(0);
                 arrayYRobo.remove(0);

                 //add the new values
                 arrayXRobo.add(xRobo);
                 arrayYRobo.add(yRobo);
             }
         }

         //robot don't has to be in the mode PAUSE -> because then the arrays would be filled with a lot of nulls
         if (arrayX.size() < 200) {
             //add the new value at the end of the list
             if(arrayX.contains(xBild) == false) {
                 arrayX.add(xBild);
                 arrayY.add(yBild);
             }
         } else {
             //remove the first value in the arrayList to add the new value at the end of the list
             if(arrayX.contains(xBild) == false) {
                 arrayX.remove(0);
                 arrayY.remove(0);

                 //add the new values
                 arrayX.add(xBild);
                 arrayY.add(yBild);
             }
         }

         //draw a new canvas
         canvas = getNewCanvas();
         for (int i = 0; i < arrayX.size(); i++) {
             xBild = arrayX.get(i);
             yBild = arrayY.get(i);

             canvas.drawCircle(xBild, yBild, 5, pinsel);
         }

         // based on the arrayLists: caculate the driven distance
         strecke = pfadberechnung(arrayXRobo, arrayYRobo);

         //update the display
         TextView pfadDistanz = (TextView) findViewById(R.id.textViewDistanceValue);

         //limit the decimal place of the float value (only two are accepted)
         NumberFormat numberFormat = new DecimalFormat("0.00");
         numberFormat.setRoundingMode(RoundingMode.DOWN);
         streckeAnzeige = numberFormat.format(strecke - 150);

         pfadDistanz.setText(streckeAnzeige + " cm");

     }


    /**
     * This method reposition the image (the car icon) every time it moved (in real-time).
     * So the user can see where the car currently is (in the parcours).
     * @param xRobo
     * @param yRobo
     */
     private void imagePositionieren(float xRobo, float yRobo){
         //find the image
         imageRobo = (ImageView) findViewById(R.id.robo);

         // set the angle of the icon true to the current angle of the robot
         if(hmiModule == null){
             angle = 0;
         }else{
             angle = (360 - hmiModule.getPosition().getAngle());
         }


         //convert the values (from values of the robot to values of the image map)
         imageX = (float) ((xRobo * SKALROBO) + (100 - BREITE_AUTO));
         imageY = (float) (((yRobo * SKALROBO) - (248 - HOEHE_AUTO)) * (-1));

         //reposition the image in the map
         imageRobo.setY(imageY);
         imageRobo.setX(imageX);
         imageRobo.setRotation(angle);


     }


    /**
     * This method create a new canvas.
     * @return canvas
     */
     @SuppressLint("NewApi")
     private Canvas getNewCanvas(){
         //create a new canvas, delete the old one
         RelativeLayout layout = (RelativeLayout) findViewById(R.id.viewLayout);

         Bitmap bitmap = Bitmap.createBitmap(1024, 552, Bitmap.Config.ARGB_8888);
         canvas = new Canvas(bitmap);
         layout.setBackground(new BitmapDrawable(bitmap));

         return canvas;
     }



    /**
     * This method calculate the driven distance.
     * The value would be shown in the right bottom corner of the display.
     * @param arrayX
     * @param arrayY
     */
     public float pfadberechnung(ArrayList<Float> arrayX, ArrayList<Float> arrayY){
         // values needed for the calculation
         float differenzX =  0, differenzY = 0;
         float teilstrecke = 0;


         // execute the calculation only, if the robot is in movement -> else the values would be incorrect
         if(hmiModule.getCurrentStatus() == IGuidance.CurrentStatus.DRIVING) {
             if (arrayX.size() > 1) {
                 xAlt = arrayX.get(arrayX.size() - 2);
                 xNeu = arrayX.get(arrayX.size() - 1);
             } else if (arrayX.size() == 1) {
                 xAlt = 0;
                 xNeu = arrayX.get(arrayX.size() - 1);
             } else if (arrayX.size() == 0) {
                 xAlt = 0;
                 xNeu = 0;
             }else{
                 System.out.println("Es ist ein Fehler aufgetreten!");
             }

             if (arrayY.size() > 1) {
                 yAlt = arrayY.get(arrayY.size() - 2);
                 yNeu = arrayY.get(arrayY.size() - 1);
             } else if (arrayY.size() == 1) {
                 yAlt = 0;
                 yNeu = arrayY.get(arrayY.size() - 1);
             } else if(arrayY.size() == 0) {
                yAlt = 0;
                yNeu = 0;
             } else{
                 System.out.println("Es ist ein Fehler aufgetreten!");
             }


             // calculate the difference
             differenzX = Math.abs(xAlt - xNeu);
             differenzY = Math.abs(yAlt - yNeu);

             //distinction of cases
             if (differenzX <= ABWEICHUNG) {
                 strecke = strecke + differenzY;
             } else if (differenzY <= ABWEICHUNG) {
                 strecke = strecke + differenzX;
             } else {
                 //use "Satz des Pythagoras" for the calculation
                 teilstrecke = (float) Math.sqrt((differenzX * differenzX) + (differenzY * differenzY));
                 strecke = strecke + teilstrecke;
             }

         }

         return strecke;

     }


    /**
     * This method is used to update the images of the sensors, if the values are in a critical area.
     * Then the normal image of the "wlan" symbol change to the attantion triangle.
      */
    public void sensorBilder() {
        double critical = 0.0;     //measurement in cm
        double distBack = hmiModule.getPosition().getDistanceBack();
        double distFront = hmiModule.getPosition().getDistanceFront();

        //images
        ImageView sensorBack = (ImageView) findViewById(R.id.wlanBack);
        ImageView sensorFront = (ImageView) findViewById(R.id.wlanFront);


        //request for a critical value
        if (distBack < critical) {
            if (backAttention == false) {
                //change the picture in back of the car icon -> attention
                sensorBack.setImageResource(R.drawable.achtung);
                Toast.makeText(this, "ACHTUNG!! Abstand wird eng!!", Toast.LENGTH_LONG).show();

                backAttention = true;
            } else {
                sensorBack.setImageResource(R.drawable.achtung);
                backAttention = true;
            }
        } else {
            //rechange the image -> "wlan" symbol (values are measured)
            sensorBack.setImageResource(R.drawable.wlan_back);

            backAttention = false;
        }

        if (distFront < critical) {
            if (frontAttention == false) {
                //change the image in front of the car icon -> attention
                sensorFront.setImageResource(R.drawable.achtung);
                Toast.makeText(this, "ACHTUNG!! Abstand wird eng!!", Toast.LENGTH_LONG).show();

                frontAttention = true;
            } else {
                sensorFront.setImageResource(R.drawable.achtung);
                frontAttention = true;
            }
        } else {
            //rechange the image -> "wlan" symbol
            sensorFront.setImageResource(R.drawable.wlan_front);

            frontAttention = false;
        }

    }


    /**
     * This method shows the detected parking slots. The user can choose one of this to start the autonomous parking process.
     */
    public void parkSlots() {
        float breite, hoehe;
        float xSet, ySet;
        float xPos, yPos;
        IAndroidHmi.ParkingSlot parkingSlot;


        if (hmiModule != null) {
            numberNew = hmiModule.getNoOfParkingSlots();
        } else {
            numberNew = 0;
        }

        //set int-values of the ID
        ArrayList<Integer> ids = new ArrayList<>();
        ids.add(0, R.id.button_ParkingSlot1);
        ids.add(1, R.id.button_ParkingSlot2);
        ids.add(2, R.id.button_ParkingSlot3);
        ids.add(3, R.id.button_ParkingSlot4);
        ids.add(4, R.id.button_ParkingSlot5);
        ids.add(5, R.id.button_ParkingSlot6);
        ids.add(6, R.id.button_ParkingSlot7);
        ids.add(7, R.id.button_ParkingSlot8);

        //search only for parking slots in detail if one is detected at least
        if (numberNew != numberOld) {
           // insert elements to the end of the array

            //Parklücke auswählen
            parkingSlot = hmiModule.getParkingSlot(numberNew - 1);

            // choose one parking slot
            if (parkingSlot.getBackBoundaryPosition().x >= 175) {
                //insert button to the right of the parcours map
                hoehe = Math.abs(parkingSlot.getBackBoundaryPosition().y - parkingSlot.getFrontBoundaryPosition().y);
                hoehe = (float) ((hoehe * 3));

                 //insert Button ParkinSlot
                RelativeLayout rl = (RelativeLayout) findViewById(R.id.layoutParkingSlots);
                Button btn = new Button(this);
                btn.setWidth(30);
                btn.setHeight((int) hoehe);
                //btn.setHeight(150);
                btn.setId(ids.get(numberNew - 1));     //ID des Buttons setzen -> ID aus Array

                //get the values of the Navigation modul
                xPos = parkingSlot.getFrontBoundaryPosition().x;
                yPos = parkingSlot.getFrontBoundaryPosition().y;

                //convert the values
                xSet = (float) ((xPos * XSKAL) + 100);
                ySet = (float) (((yPos * YSKAL) - 263) * (-1));

                //respect the displacement
                xSet = xSet + VERSCHIEBUNG;

                btn.setX(xSet);
                btn.setY(ySet);

                //set the color of the button
                if (parkingSlot.getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING) {
                    btn.setBackgroundColor(Color.GREEN);
                    btn.setTextColor(Color.BLACK);
                    status = "suitable";

                    btn.setText("suitable");
                } else {
                    btn.setBackgroundColor(Color.RED);
                    btn.setTextColor(Color.DKGRAY);
                    status = "notsuitable";

                    btn.setText("not suitable");
                }

                //add the parking slot to the button
                btn.setHint(parkingSlot.getID());

                //change the label
                btn.setText(String.valueOf(numberNew));

                //set OnClickListener
                btn.setOnClickListener(new View.OnClickListener() {
                    public void onClick(View view) {
                        Button btn = (Button) findViewById(view.getId());

                         //if the button is enable, the status of the parking slot is suitable
                        if (btn.getCurrentTextColor() == Color.BLACK) {
                            //set ParkSlot
                            //convert String in Integer
                            int id = Integer.parseInt(btn.getHint().toString());
                            hmiModule.setSelectedParkingSlot(id);
                            System.out.println("Status Parklücke: " + hmiModule.getParkingSlot(id).getParkingSlotStatus());

                            //change the mode to ParkThis and start the autonomous parking process
                            hmiModule.setMode(INxtHmi.Mode.PARK_THIS);

                            //change the label of the button
                            final ToggleButton scoutButton = (ToggleButton) findViewById(R.id.toggleMode);
                            scoutButton.setEnabled(false);

                            //Ausgabe für den Benutzer
                            Toast.makeText(MainActivity.this, "Einparkvorgang eingeleitet.", Toast.LENGTH_LONG).show();

                        } else if (btn.isEnabled() == false) {
                            Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_LONG).show();
                        } else {
                            Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                        }
                    }
                });

                rl.addView(btn);

            } else if (parkingSlot.getBackBoundaryPosition().y < 20) {
                //insert Button underneath the robot (parcours)
                breite = Math.abs(parkingSlot.getBackBoundaryPosition().x - parkingSlot.getFrontBoundaryPosition().x);
                breite = (float) (breite * 2.5);

                //insert Button ParkinSlot
                RelativeLayout rl = (RelativeLayout) findViewById(R.id.layoutParkingSlots);
                Button btn = new Button(this);
                btn.setWidth((int) breite);
                //btn.setWidth(375);
                btn.setHeight(30);
                btn.setId(ids.get(numberNew - 1));

                //get the values
                xPos = parkingSlot.getBackBoundaryPosition().x;
                yPos = parkingSlot.getBackBoundaryPosition().y;
                //convert the values
                xSet = (float) ((xPos * XSKAL) + 100);
                ySet = (float) (((yPos * YSKAL) - 253) * (-1));

                //respect the displacement
                ySet = ySet + VERSCHIEBUNG - 5;

                btn.setX(xSet);
                btn.setY(ySet);

                //set the color of the button
                if (parkingSlot.getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING) {
                    btn.setBackgroundColor(Color.GREEN);
                    btn.setTextColor(Color.BLACK);
                    status = "suitable";

                    btn.setText("suitable");
                } else {
                    btn.setBackgroundColor(Color.RED);
                    btn.setTextColor(Color.DKGRAY);
                    status = "notsuitable";

                    btn.setText("not suitable");
                }

                //add the parking slot to the button
                btn.setHint(parkingSlot.getID());

                //change the label of the button
                btn.setText(String.valueOf(numberNew));

                //set OnClickListener
                btn.setOnClickListener(new View.OnClickListener() {
                    public void onClick(View view) {
                        Button btn = (Button) findViewById(view.getId());

                        //if the button is enabled, the status of the parking slot is suitable
                        if (btn.getCurrentTextColor() == Color.BLACK) {
                            //set parking slot
                            //convert String in Integer
                            int id = Integer.parseInt(btn.getHint().toString());
                            hmiModule.setSelectedParkingSlot(id);
                            System.out.println("Status Parklücke: " + hmiModule.getParkingSlot(id).getParkingSlotStatus());

                            //change to the mode ParkThis and start the autonomous parking process
                            hmiModule.setMode(INxtHmi.Mode.PARK_THIS);

                            //change the label of the button
                            final ToggleButton scoutButton = (ToggleButton) findViewById(R.id.toggleMode);
                            scoutButton.setEnabled(false);

                            //Ausgabe für den Benutzer
                            Toast.makeText(MainActivity.this, "Einparkvorgang eingeleitet.", Toast.LENGTH_SHORT).show();

                        } else if (btn.getCurrentTextColor() == Color.DKGRAY) {
                            Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_LONG).show();
                        } else {
                            Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                        }
                    }
                });

                rl.addView(btn);

            } else if (parkingSlot.getBackBoundaryPosition().y >= 20) {
                //insert Button above the robot (parcours)
                breite = Math.abs(parkingSlot.getBackBoundaryPosition().x - parkingSlot.getFrontBoundaryPosition().x);
                breite = (float) (breite * 2.5);

                //insert Button ParkinSlot
                RelativeLayout rl = (RelativeLayout) findViewById(R.id.layoutParkingSlots);
                Button btn = new Button(this);
                btn.setWidth((int) breite);
                //btn.setWidth(200);
                btn.setHeight(30);
                btn.setId(ids.get(numberNew - 1));

                //get the values
                xPos = parkingSlot.getBackBoundaryPosition().x;
                yPos = parkingSlot.getBackBoundaryPosition().y;
                //convert the values
                xSet = (float) ((xPos * XSKAL) + 100);
                ySet = (float) (((yPos * YSKAL) - 273) * (-1));

                //displacement
                ySet = ySet - 60;

                //another displacement
                ySet = ySet - VERSCHIEBUNG;

                btn.setX(xSet);
                btn.setY(ySet);

                 //set the color of the button
                if (parkingSlot.getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING) {
                    btn.setBackgroundColor(Color.GREEN);
                    btn.setTextColor(Color.BLACK);
                    status = "suitable";

                    btn.setText("suitaböe");
                } else {
                    btn.setBackgroundColor(Color.RED);
                    btn.setTextColor(Color.DKGRAY);
                    status = "notsuitable";

                    btn.setText("not suitable");
                }

                //add the ParkSlot to Button
                btn.setHint(parkingSlot.getID());

                //change the label of the button
                btn.setText(String.valueOf(numberNew));

                //set OnClickListener
                btn.setOnClickListener(new View.OnClickListener() {
                    public void onClick(View view) {
                        Button btn = (Button) findViewById(view.getId());

                        //if the button is enable, the status is suitable
                        if (btn.getCurrentTextColor() == Color.BLACK) {
                            //set ParkSlot
                            //convert String in Integer
                            int id = Integer.parseInt(btn.getHint().toString());
                            hmiModule.setSelectedParkingSlot(id);
                            System.out.println("Status Parklücke: " + hmiModule.getParkingSlot(id).getParkingSlotStatus());

                            //change to the mode ParkThis and start the autonomous parking process
                            hmiModule.setMode(INxtHmi.Mode.PARK_THIS);

                            //change the apperance of the button
                            final ToggleButton scoutButton = (ToggleButton) findViewById(R.id.toggleMode);
                            scoutButton.setEnabled(false);

                            //Ausgabe für den Benutzer
                            Toast.makeText(MainActivity.this, "Einparkvorgang eingeleitet.", Toast.LENGTH_SHORT).show();

                        } else if (btn.getCurrentTextColor() == Color.DKGRAY) {
                            Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_LONG).show();
                        } else {
                            Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                        }
                    }
                });

                rl.addView(btn);
            }
        }

        //set new numberOld
        numberOld = numberNew;
    }


    /**
     * This method is used to show the functionality of the method withut the data of the Navigation module.
     * Only needed for testing.
     */
    public void testenPS() {
        float breite, hoehe;
        float xSet, ySet;
        float xPos, yPos;
        IAndroidHmi.ParkingSlot parkingSlot;

        //List where all parking slots are insert
        ArrayList<IAndroidHmi.ParkingSlot> parkSlotList = new ArrayList<>();

        //create static parking slots
        IAndroidHmi.ParkingSlot parkSlot1 = new IAndroidHmi.ParkingSlot(1, new PointF(20, 0), new PointF(135, 0), IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING);
        IAndroidHmi.ParkingSlot parkSlot2 = new IAndroidHmi.ParkingSlot(2, new PointF(180, 15), new PointF(180, 45), IAndroidHmi.ParkingSlot.ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING);
        IAndroidHmi.ParkingSlot parkSlot3 = new IAndroidHmi.ParkingSlot(3, new PointF(100, 30), new PointF(120, 30), IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING);
        IAndroidHmi.ParkingSlot parkSlot4 = new IAndroidHmi.ParkingSlot(3, new PointF(60, 30), new PointF(80, 30), IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING);

        //add parking slots to ArrayList
        parkSlotList.add(parkSlot1);
        parkSlotList.add(parkSlot2);
        parkSlotList.add(parkSlot3);
        parkSlotList.add(parkSlot4);

        //ArrayList with the static ID of the parking slots
        // set int-values of the IDs
        ArrayList<Integer> ids = new ArrayList<>();
        ids.add(0, R.id.PS_statisch_1);
        ids.add(1, R.id.PS_statisch_2);
        ids.add(2, R.id.PS_statisch_3);
        ids.add(3, R.id.PS_statisch_4);

        // search only for parking slot details, if one is detected at least
        if (parkSlotList.size() != 0) {
            for (int i = 0; i < parkSlotList.size(); i++) {

                //choose one parking slot
                parkingSlot = parkSlotList.get(i);

                //choose position
                if (parkingSlot.getBackBoundaryPosition().x >= 175) {
                    // insert Button to the right of the parcours
                    hoehe = Math.abs(parkingSlot.getBackBoundaryPosition().y - parkingSlot.getFrontBoundaryPosition().y);
                    hoehe = (float) ((hoehe * 3) );


                    //insert Button ParkinSlot
                    RelativeLayout rl = (RelativeLayout) findViewById(R.id.layoutParkingSlots);
                    Button btn = new Button(this);
                    btn.setWidth(30);
                    btn.setHeight((int) hoehe);
                    //btn.setHeight(150);
                    btn.setId(ids.get(i));     //set ID of the Button -> ID aus Array

                    //get the values
                    xPos = parkingSlot.getFrontBoundaryPosition().x;
                    yPos = parkingSlot.getFrontBoundaryPosition().y;

                    //convert the values
                    xSet = (float) ((xPos * XSKAL) + 100);
                    ySet = (float) (((yPos * YSKAL) - 263) * (-1));

                    //respect the displacement
                    xSet = xSet + VERSCHIEBUNG;

                    btn.setX(xSet);
                    btn.setY(ySet);

                    //set the colour of the button
                    if (parkingSlot.getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING) {
                        btn.setBackgroundColor(Color.GREEN);
                        btn.setTextColor(Color.BLACK);
                        status = "suitable";

                        btn.setText("suitable");
                    } else {
                        btn.setBackgroundColor(Color.RED);
                        btn.setTextColor(Color.DKGRAY);
                        status = "notsuitable";

                        btn.setText("not suitable");
                    }

                    //change the label of the button
                    btn.setText(String.valueOf(i+1));
                    btn.setVisibility(View.INVISIBLE);

                    //set OnClickListener
                    btn.setOnClickListener(new View.OnClickListener() {
                        public void onClick(View view) {
                            Button btn = (Button) findViewById(view.getId());

                            // if the button is enable, the status is suitable
                            if (btn.getCurrentTextColor() == Color.BLACK) {
                                // set the parking slot -> but here doesn't exist any hmiModule (so you can't do this step)

                                if (hmiModule != null) {
                                    //set ParkSlot
                                    //convert String in Integer
                                    int id = Integer.parseInt(btn.getHint().toString());
                                    hmiModule.setSelectedParkingSlot(id);

                                    // change to the mode ParkThis and start the autonomous parking process
                                    hmiModule.setMode(INxtHmi.Mode.PARK_THIS);

                                    //Ausgabe für den Benutzer
                                    Toast.makeText(MainActivity.this, "Einparkvorgang eingeleitet.", Toast.LENGTH_SHORT).show();
                                }else{
                                    Toast.makeText(MainActivity.this, "Parklücke " + btn.getText() + " wurde angeklickt.", Toast.LENGTH_SHORT).show();
                                }

                                //change the label of the button
                                final ToggleButton scoutButton = (ToggleButton) findViewById(R.id.toggleMode);
                                scoutButton.setEnabled(false);

                            } else if (btn.getCurrentTextColor() == Color.DKGRAY) {
                                Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_SHORT).show();
                            } else {
                                Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                            }
                        }
                    });

                    rl.addView(btn);

                } else if (parkingSlot.getBackBoundaryPosition().y < 20) {
                    //insert Button underneath the robot (parcours)
                    breite = Math.abs(parkingSlot.getBackBoundaryPosition().x - parkingSlot.getFrontBoundaryPosition().x);
                    breite = (float) (breite * 2.5);

                    //insert Button ParkinSlot
                    RelativeLayout rl = (RelativeLayout) findViewById(R.id.layoutParkingSlots);
                    Button btn = new Button(this);
                    btn.setWidth((int) breite);
                    //btn.setWidth(375);
                    btn.setHeight(30);
                    btn.setId(ids.get(i));

                    //get the values
                    xPos = parkingSlot.getBackBoundaryPosition().x;
                    yPos = parkingSlot.getBackBoundaryPosition().y;
                    //convert the values
                    xSet = (float) ((xPos * XSKAL) + 100);
                    ySet = (float) (((yPos * YSKAL) - 253) * (-1));

                    //respect the displacement
                    ySet = ySet + VERSCHIEBUNG - 5;

                    btn.setX(xSet);
                    btn.setY(ySet);

                    //set the colour of the button
                    if (parkingSlot.getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING) {
                        btn.setBackgroundColor(Color.GREEN);
                        btn.setTextColor(Color.BLACK);
                        status = "suitable";

                        btn.setText("suitable");
                    } else {
                        btn.setBackgroundColor(Color.RED);
                        btn.setTextColor(Color.DKGRAY);
                        status = "notsuitable";

                        btn.setText("not suitable");
                    }

                    //change the label of the button
                    btn.setText(String.valueOf(i+1));
                    btn.setVisibility(View.INVISIBLE);

                    //set OnClickListener
                    btn.setOnClickListener(new View.OnClickListener() {
                        public void onClick(View view) {
                            Button btn = (Button) findViewById(view.getId());

                            //if the button is enable, the status is suitable
                            if (btn.getCurrentTextColor() == Color.BLACK) {
                                // set parking slot -> it doesn't exist any hmiModule (so you can't do this step)

                                if (hmiModule != null) {
                                    //set ParkSlot
                                    //convert String in Integer
                                    int id = Integer.parseInt(btn.getHint().toString());
                                    hmiModule.setSelectedParkingSlot(id);

                                    // change to the mode ParkThis and start the autonomous parking process
                                    hmiModule.setMode(INxtHmi.Mode.PARK_THIS);

                                    //Ausgabe für den Benutzer
                                    Toast.makeText(MainActivity.this, "Einparkvorgang eingeleitet.", Toast.LENGTH_SHORT).show();
                                }else{
                                    Toast.makeText(MainActivity.this, "Parklücke " + btn.getText() + " wurde angeklickt.", Toast.LENGTH_SHORT).show();
                                }

                                //change the label of the button
                                final ToggleButton scoutButton = (ToggleButton) findViewById(R.id.toggleMode);
                                scoutButton.setEnabled(false);

                            } else if (btn.getCurrentTextColor() == Color.DKGRAY) {
                                Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_SHORT).show();
                            } else {
                                Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                            }
                        }
                    });

                    rl.addView(btn);

                } else if (parkingSlot.getBackBoundaryPosition().y >= 20) {
                    //insert Button above the robot (parcours)
                    breite = Math.abs(parkingSlot.getBackBoundaryPosition().x - parkingSlot.getFrontBoundaryPosition().x);
                    breite = (float) (breite * 2.5);

                    //insert Button ParkinSlot
                    RelativeLayout rl = (RelativeLayout) findViewById(R.id.layoutParkingSlots);
                    Button btn = new Button(this);
                    btn.setWidth((int) breite);
                    //btn.setWidth(200);
                    btn.setHeight(30);
                    btn.setId(ids.get(i));

                    //get the values
                    xPos = parkingSlot.getBackBoundaryPosition().x;
                    yPos = parkingSlot.getBackBoundaryPosition().y;
                    //convert the values
                    xSet = (float) ((xPos * XSKAL) + 100);
                    ySet = (float) (((yPos * YSKAL) - 273) * (-1));

                    //displacement
                    ySet = ySet - 60;

                    //another displacement
                    ySet = ySet - VERSCHIEBUNG;

                    btn.setX(xSet);
                    btn.setY(ySet);


                    //set the colour of the button
                    if (parkingSlot.getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING) {
                        btn.setBackgroundColor(Color.GREEN);
                        btn.setTextColor(Color.BLACK);
                        status = "suitable";

                        btn.setText("suitable");
                    } else {
                        btn.setBackgroundColor(Color.RED);
                        btn.setTextColor(Color.DKGRAY);
                        status = "notsuitable";

                        btn.setText("not suitable");
                    }

                    //change the label of the button
                    btn.setText(String.valueOf(i+1));
                    btn.setVisibility(View.INVISIBLE);

                    //set OnClickListener
                    btn.setOnClickListener(new View.OnClickListener() {
                        public void onClick(View view) {
                            Button btn = (Button) findViewById(view.getId());

                            //if the button is enable, the status is suitable
                            if (btn.getCurrentTextColor() == Color.BLACK) {
                                // set the parking slot -> it doesn't exist any hmiModule (so you can't do this step)

                                if (hmiModule != null) {
                                    //set ParkSlot
                                    //convert String in Integer
                                    int id = Integer.parseInt(btn.getHint().toString());
                                    hmiModule.setSelectedParkingSlot(id);

                                    // change to the mode ParkThis and start the autonomous parking process
                                    hmiModule.setMode(INxtHmi.Mode.PARK_THIS);

                                    //Ausgabe für den Benutzer
                                    Toast.makeText(MainActivity.this, "Einparkvorgang eingeleitet.", Toast.LENGTH_SHORT).show();
                                }else{
                                    Toast.makeText(MainActivity.this, "Parklücke " + btn.getText() + " wurde angeklickt.", Toast.LENGTH_SHORT).show();
                                }

                                //change the label of the button
                                final ToggleButton scoutButton = (ToggleButton) findViewById(R.id.toggleMode);
                                scoutButton.setEnabled(false);

                            } else if (btn.getCurrentTextColor() == Color.DKGRAY) {
                                Toast.makeText(MainActivity.this, "Diese Parklücke ist zum Einparken nicht geeignet.", Toast.LENGTH_SHORT).show();
                            } else {
                                Toast.makeText(MainActivity.this, "Es ist ein Fehler aufgetreten.", Toast.LENGTH_LONG).show();
                            }
                        }
                    });

                rl.addView(btn);
            }
        }
    }
}
}
