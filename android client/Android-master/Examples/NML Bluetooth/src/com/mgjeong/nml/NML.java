package com.mgjeong.nml;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.UUID;

import android.app.Activity;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
//import android.widget.Toast;
import android.widget.RelativeLayout;

public class NML extends Activity implements View.OnClickListener, View.OnTouchListener {
	private final static String TAG = NML.class.getSimpleName();

	public static final String EXTRAS_DEVICE = "EXTRAS_DEVICE";
	private TextView tv = null;
	private EditText et = null;
	private Button btn = null;
	private Button btn_up = null;
	private Button btn_down = null;
	private Button btn_left = null;
	private Button btn_right = null;
	private RelativeLayout mg_bong = null;
	private ImageView mg_bong_img = null;
	private String mDeviceName;
	private String mDeviceAddress;
	private RBLService mBluetoothLeService;
	private Map<UUID, BluetoothGattCharacteristic> map = new HashMap<UUID, BluetoothGattCharacteristic>();
	private FileOutputStream fos;
	private String dirPath;
	private String testStr = "";
	private File savefile;
	private int no;

	private int[] posXY = new int[2];

	private final ServiceConnection mServiceConnection = new ServiceConnection() {

		@Override
		public void onServiceConnected(ComponentName componentName,
									   IBinder service) {
			mBluetoothLeService = ((RBLService.LocalBinder) service)
					.getService();
			if (!mBluetoothLeService.initialize()) {
				Log.e(TAG, "Unable to initialize Bluetooth");
				finish();
			}

			//save data to text file...
			dirPath = "/storage/emulated/0/log";
			//String dirPath = Environment.getDataDirectory().getAbsolutePath();
			//dirPath = dirPath + "/log";
			File file = new File(dirPath);
			if (!file.exists()) {
				file.mkdir();
			}
			for (int i = 0; i < 10; i++) {
				savefile = new File(dirPath + "/text" + Integer.toString(i) + ".txt");
				if (savefile.exists()) {
					continue;
				} else {
					break;
				}
			}
			try {
				fos = new FileOutputStream(savefile);
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}
			no = 1;


			// Automatically connects to the device upon successful start-up
			// initialization.
			mBluetoothLeService.connect(mDeviceAddress);
		}

		@Override
		public void onServiceDisconnected(ComponentName componentName) {
			mBluetoothLeService = null;
		}
	};

	private final BroadcastReceiver mGattUpdateReceiver = new BroadcastReceiver() {
		@Override
		public void onReceive(Context context, Intent intent) {
			final String action = intent.getAction();

			if (RBLService.ACTION_GATT_DISCONNECTED.equals(action)) {
			} else if (RBLService.ACTION_GATT_SERVICES_DISCOVERED
					.equals(action)) {
				getGattService(mBluetoothLeService.getSupportedGattService());
			} else if (RBLService.ACTION_DATA_AVAILABLE.equals(action)) {
				displayData(intent.getByteArrayExtra(RBLService.EXTRA_DATA));
			}
		}
	};

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.second);

		tv = (TextView) findViewById(R.id.textView);
		tv.setMovementMethod(ScrollingMovementMethod.getInstance());

		Intent intent = getIntent();

		mDeviceAddress = intent.getStringExtra(Device.EXTRA_DEVICE_ADDRESS);
		mDeviceName = intent.getStringExtra(Device.EXTRA_DEVICE_NAME);

		getActionBar().setTitle(mDeviceName);
		getActionBar().setDisplayHomeAsUpEnabled(true);

		Intent gattServiceIntent = new Intent(this, RBLService.class);
		bindService(gattServiceIntent, mServiceConnection, BIND_AUTO_CREATE);


		et = (EditText) findViewById(R.id.editText);
		btn = (Button) findViewById(R.id.send);
		btn_up = (Button) findViewById(R.id.up);        //"1" - 49
		btn_down = (Button) findViewById(R.id.down);   //"2" - 50
		btn_left = (Button) findViewById(R.id.left);   //"3" - 51
		btn_right = (Button) findViewById(R.id.right); //"4" - 52
		mg_bong = (RelativeLayout) findViewById(R.id.mgbong);
		mg_bong_img.getLocationInWindow(posXY);

		btn.setOnClickListener(this);
		btn_up.setOnClickListener(this);
		btn_down.setOnClickListener(this);
		btn_left.setOnClickListener(this);
		btn_right.setOnClickListener(this);

		//update 1.3v
		btn_up.setOnTouchListener(this);
		btn_down.setOnTouchListener(this);
		btn_left.setOnTouchListener(this);
		btn_right.setOnTouchListener(this);
		mg_bong.setOnTouchListener(this);
	}

	@Override
	public boolean onTouch(View v, MotionEvent event) {
		BluetoothGattCharacteristic characteristic = map
				.get(RBLService.UUID_BLE_SHIELD_TX);
		int x;
		int y;
		byte b = 0x00;
		byte[] tmp;
		byte[] tx;
		String str;

		switch (v.getId()) {
			case R.id.mgbong:
				switch (event.getAction()) {
					case MotionEvent.ACTION_DOWN:
//						x = posXY[0];
//						y = posXY[1];
						x = (int) event.getRawX() - posXY[0];
						y = (int) event.getRawY() - posXY[1];
						str = Integer.toString(x) + ", " + Integer.toString(y) + "\n";
						tmp = str.getBytes();
						tx = new byte[tmp.length + 1];
						tx[0] = b;
						for (int i = 1; i < tmp.length + 1; i++) {
							tx[i] = tmp[i - 1];
						}
						characteristic.setValue(tx);
						mBluetoothLeService.writeCharacteristic(characteristic);
						//Toast.makeText(this, "DOWN", Toast.LENGTH_SHORT).show();
						break;
					case MotionEvent.ACTION_MOVE:
//						x = posXY[0];
//						y = posXY[1];
						x = (int) event.getRawX() - posXY[0];
						y = (int) event.getRawY() - posXY[1];
						str = Integer.toString(x) + ", " + Integer.toString(y) + "\n";
						tmp = str.getBytes();
						tx = new byte[tmp.length + 1];
						tx[0] = b;
						for (int i = 1; i < tmp.length + 1; i++) {
							tx[i] = tmp[i - 1];
						}
						characteristic.setValue(tx);
						mBluetoothLeService.writeCharacteristic(characteristic);
						//Toast.makeText(this, "DOWN", Toast.LENGTH_SHORT).show();
						break;
				}
			case R.id.up:
				switch (event.getAction()) {
					case MotionEvent.ACTION_DOWN:
						str = "1";
						tmp = str.getBytes();
						tx = new byte[tmp.length + 1];
						tx[0] = b;
						for (int i = 1; i < tmp.length + 1; i++) {
							tx[i] = tmp[i - 1];
						}
						characteristic.setValue(tx);
						mBluetoothLeService.writeCharacteristic(characteristic);
						//Toast.makeText(this, "DOWN", Toast.LENGTH_SHORT).show();
						break;
					case MotionEvent.ACTION_MOVE:
						str = "1";
						tmp = str.getBytes();
						tx = new byte[tmp.length + 1];
						tx[0] = b;
						for (int i = 1; i < tmp.length + 1; i++) {
							tx[i] = tmp[i - 1];
						}
						characteristic.setValue(tx);
						mBluetoothLeService.writeCharacteristic(characteristic);
						//Toast.makeText(this, "DOWN", Toast.LENGTH_SHORT).show();
						break;

					case MotionEvent.ACTION_UP:
						break;
				}
				break;
			case R.id.down:
				switch (event.getAction()) {
					case MotionEvent.ACTION_DOWN:
						str = "2";
						tmp = str.getBytes();
						tx = new byte[tmp.length + 1];
						tx[0] = b;
						for (int i = 1; i < tmp.length + 1; i++) {
							tx[i] = tmp[i - 1];
						}
						//Toast.makeText(this, Integer.toString(Character.digit(tx[1], 10)), Toast.LENGTH_SHORT).show();
						characteristic.setValue(tx);
						mBluetoothLeService.writeCharacteristic(characteristic);
						break;
					case MotionEvent.ACTION_MOVE:
						str = "2";
						tmp = str.getBytes();
						tx = new byte[tmp.length + 1];
						tx[0] = b;
						for (int i = 1; i < tmp.length + 1; i++) {
							tx[i] = tmp[i - 1];
						}
						//Toast.makeText(this, Integer.toString(Character.digit(tx[1], 10)), Toast.LENGTH_SHORT).show();
						characteristic.setValue(tx);
						mBluetoothLeService.writeCharacteristic(characteristic);
						break;
					case MotionEvent.ACTION_UP:
						break;
				}
				break;
			case R.id.left:
				switch (event.getAction()) {
					case MotionEvent.ACTION_DOWN:
						str = "3";
						tmp = str.getBytes();
						tx = new byte[tmp.length + 1];
						tx[0] = b;
						for (int i = 1; i < tmp.length + 1; i++) {
							tx[i] = tmp[i - 1];
						}
						//Toast.makeText(this, Integer.toString(Character.digit(tx[1], 10)), Toast.LENGTH_SHORT).show();
						characteristic.setValue(tx);
						mBluetoothLeService.writeCharacteristic(characteristic);

						break;
					case MotionEvent.ACTION_MOVE:
						str = "3";
						tmp = str.getBytes();
						tx = new byte[tmp.length + 1];
						tx[0] = b;
						for (int i = 1; i < tmp.length + 1; i++) {
							tx[i] = tmp[i - 1];
						}
						//Toast.makeText(this, Integer.toString(Character.digit(tx[1], 10)), Toast.LENGTH_SHORT).show();
						characteristic.setValue(tx);
						mBluetoothLeService.writeCharacteristic(characteristic);
						break;
					case MotionEvent.ACTION_UP:
						break;
				}
				break;
			case R.id.right:
				switch (event.getAction()) {
					case MotionEvent.ACTION_DOWN:
						str = "4";
						tmp = str.getBytes();
						tx = new byte[tmp.length + 1];
						tx[0] = b;
						for (int i = 1; i < tmp.length + 1; i++) {
							tx[i] = tmp[i - 1];
						}
						//Toast.makeText(this, Integer.toString(Character.digit(tx[1], 10)), Toast.LENGTH_SHORT).show();
						characteristic.setValue(tx);
						mBluetoothLeService.writeCharacteristic(characteristic);

						break;
					case MotionEvent.ACTION_MOVE:
						str = "4";
						tmp = str.getBytes();
						tx = new byte[tmp.length + 1];
						tx[0] = b;
						for (int i = 1; i < tmp.length + 1; i++) {
							tx[i] = tmp[i - 1];
						}
						//Toast.makeText(this, Integer.toString(Character.digit(tx[1], 10)), Toast.LENGTH_SHORT).show();
						characteristic.setValue(tx);
						mBluetoothLeService.writeCharacteristic(characteristic);

						break;
					case MotionEvent.ACTION_UP:
						break;
				}
				break;
		}
		return false;
	}

	@Override
	public void onClick(View v) {
		BluetoothGattCharacteristic characteristic = map
				.get(RBLService.UUID_BLE_SHIELD_TX);
		byte b = 0x00;
		byte[] tmp;
		byte[] tx;
		String str;
		switch (v.getId()) {
			case R.id.up:
				str = "1";
				tmp = str.getBytes();
				tx = new byte[tmp.length + 1];
				tx[0] = b;
				for (int i = 1; i < tmp.length + 1; i++) {
					tx[i] = tmp[i - 1];
				}
				characteristic.setValue(tx);
				mBluetoothLeService.writeCharacteristic(characteristic);
				break;
			case R.id.down:
				str = "2";
				tmp = str.getBytes();
				tx = new byte[tmp.length + 1];
				tx[0] = b;
				for (int i = 1; i < tmp.length + 1; i++) {
					tx[i] = tmp[i - 1];
				}
				//Toast.makeText(this, Integer.toString(Character.digit(tx[1], 10)), Toast.LENGTH_SHORT).show();
				characteristic.setValue(tx);
				mBluetoothLeService.writeCharacteristic(characteristic);
				break;
			case R.id.left:
				str = "3";
				tmp = str.getBytes();
				tx = new byte[tmp.length + 1];
				tx[0] = b;
				for (int i = 1; i < tmp.length + 1; i++) {
					tx[i] = tmp[i - 1];
				}
				Toast.makeText(this, Integer.toString(Character.digit(tx[1], 10)), Toast.LENGTH_SHORT).show();

				characteristic.setValue(tx);
				mBluetoothLeService.writeCharacteristic(characteristic);
				break;
			case R.id.right:
				str = "4";
				tmp = str.getBytes();

				tx = new byte[tmp.length + 1];
				tx[0] = b;
				for (int i = 1; i < tmp.length + 1; i++) {
					tx[i] = tmp[i - 1];
				}
				//Toast.makeText(this, tx, )
				characteristic.setValue(tx);
				mBluetoothLeService.writeCharacteristic(characteristic);
				break;
			case R.id.send:
				str = et.getText().toString();
				tmp = str.getBytes();
				tx = new byte[tmp.length + 1];
				tx[0] = b;
				for (int i = 1; i < tmp.length + 1; i++) {
					tx[i] = tmp[i - 1];
				}

				characteristic.setValue(tx);
				mBluetoothLeService.writeCharacteristic(characteristic);

				et.setText(" ");
				break;
		}
	}


	@Override
	protected void onResume() {
		super.onResume();

		registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		if (item.getItemId() == android.R.id.home) {
			mBluetoothLeService.disconnect();
			mBluetoothLeService.close();

			System.exit(0);
		}

		return super.onOptionsItemSelected(item);
	}

	@Override
	protected void onStop() {
		try {
			fos.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		super.onStop();

	}

	@Override
	protected void onDestroy() {
		try {
			fos.close();
		} catch (IOException e) {
			e.printStackTrace();
		}

		super.onDestroy();

		mBluetoothLeService.disconnect();
		mBluetoothLeService.close();

		System.exit(0);
	}

	private void displayData(byte[] byteArray) {

		if (byteArray != null) {
			String data = new String(byteArray);
			if(no==1){
				tv.append("Start!!!");
			}

			tv.append(data);
			final int scrollAmount = tv.getLayout().getLineTop(
					tv.getLineCount())
					- tv.getHeight();

			// if there is no need to scroll, scrollAmount will be <=0
			if (scrollAmount > 0)
				tv.scrollTo(0, scrollAmount);
			else
				tv.scrollTo(0, 0);

			try {
				//testStr= testStr+ "\ndata\n" + data;
				testStr = no + "no " + data;
				fos.write((testStr).getBytes());
				no++;
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	private void getGattService(BluetoothGattService gattService) {
		if (gattService == null)
			return;

		BluetoothGattCharacteristic characteristic = gattService
				.getCharacteristic(RBLService.UUID_BLE_SHIELD_TX);
		map.put(characteristic.getUuid(), characteristic);

		BluetoothGattCharacteristic characteristicRx = gattService
				.getCharacteristic(RBLService.UUID_BLE_SHIELD_RX);
		mBluetoothLeService.setCharacteristicNotification(characteristicRx,
				true);
		mBluetoothLeService.readCharacteristic(characteristicRx);
	}

	private static IntentFilter makeGattUpdateIntentFilter() {
		final IntentFilter intentFilter = new IntentFilter();

		intentFilter.addAction(RBLService.ACTION_GATT_CONNECTED);
		intentFilter.addAction(RBLService.ACTION_GATT_DISCONNECTED);
		intentFilter.addAction(RBLService.ACTION_GATT_SERVICES_DISCOVERED);
		intentFilter.addAction(RBLService.ACTION_DATA_AVAILABLE);

		return intentFilter;
	}

}
