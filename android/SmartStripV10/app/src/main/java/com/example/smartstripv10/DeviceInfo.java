package com.example.smartstripv10;

import java.io.Serializable;
import android.os.Parcelable;
import android.os.Parcel;
import java.util.Random;

public class DeviceInfo implements Parcelable, Serializable {

    public String Name;
    public int DevID;
    public int Threshold;
    public int OnVal;
    public int OffVal;
    public int OffCount;
    public int OnCount;
    public int Status;      // 0=Off, 1=On, 2=Smart
    public int TeachScore;
    public int MinOn;
    public int MaxOff;
    public int Suffex;

    public DeviceInfo() {
        Name = "Blank Outlet";
        Threshold = -1;
        OnVal = -1;
        OffVal = -1;
        OffCount = 0;
        OnCount = 0;
        Status = 1;
        TeachScore = 0;
        MaxOff = 0;
        MinOn = 1024;

        Random rand = new Random();
        DevID = rand.nextInt();
    }

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel parcel, int i) {
        parcel.writeString(Name);
        parcel.writeInt(DevID);
        parcel.writeInt(Threshold);
        parcel.writeInt(OnVal);
        parcel.writeInt(OffVal);
        parcel.writeInt(OffCount);
        parcel.writeInt(OnCount);
        parcel.writeInt(Status);
        parcel.writeInt(TeachScore);
        parcel.writeInt(MinOn);
        parcel.writeInt(MaxOff);
        parcel.writeInt(Suffex);
    }

}
