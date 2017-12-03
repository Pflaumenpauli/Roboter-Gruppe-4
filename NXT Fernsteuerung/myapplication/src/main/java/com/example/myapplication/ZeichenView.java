package com.example.myapplication;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.widget.ImageView;
import android.content.Context;
import android.util.AttributeSet;

/**
 * Created by bi on 03.12.17.
 */

public class ZeichenView extends ImageView {

    public ZeichenView(Context context, AttributeSet attrs){
        super(context, attrs);
    }

    public ZeichenView(Context context){
        super(context);
    }

    @Override
    protected void onDraw(Canvas canvas){
        super.onDraw(canvas);

        // 1. Paint-Objekt für Blauton und Linienbreite von 5 Pixeln erzeugen
        Paint pinsel = new Paint();
        pinsel.setColor(Color.rgb(64, 64, 255));
        pinsel.setStrokeWidth(5);

        //Diagonale durch Leinwand zeichnen
        canvas.drawLine(0, 0, getWidth(), getHeight(), pinsel);

        canvas.drawCircle(306, 273, 5, pinsel);
        canvas.drawCircle(678, 273, 5, pinsel);
        canvas.drawCircle(678, 150, 5, pinsel);
        canvas.drawCircle(616, 150, 5, pinsel);
        canvas.drawCircle(616, 211, 5, pinsel);
        canvas.drawCircle(368, 211, 5, pinsel);
        canvas.drawCircle(368, 150, 5, pinsel);
        canvas.drawCircle(306, 150, 5, pinsel);
    }
}

