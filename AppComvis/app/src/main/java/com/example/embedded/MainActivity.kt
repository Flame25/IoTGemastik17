package com.example.embedded

import android.os.Bundle
import android.os.CountDownTimer
import android.widget.Toast
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import com.example.embedded.databinding.ActivityMainBinding
import com.google.firebase.database.DatabaseReference
import com.google.firebase.database.FirebaseDatabase

class MainActivity : AppCompatActivity() {
    private lateinit var binding: ActivityMainBinding
    private lateinit var database:DatabaseReference
    private lateinit var database_ref2:DatabaseReference
    private lateinit var database_ref3:DatabaseReference
    private lateinit var timer: CountDownTimer

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        // Initialize the timer with a duration of 60 seconds and interval of 1 second
        timer = object : CountDownTimer(Long.MAX_VALUE, 3000) {
            override fun onTick(millisUntilFinished: Long) {
                readData()
            }

            override fun onFinish() {
                // Update TextView when the timer finishes;
            }
        }

        // Start the timer
        timer.start()

    }

    private fun readData(){
        database = FirebaseDatabase.getInstance().getReference("/")
        database.child("label").get().addOnSuccessListener {
            if(it.exists()){
                val label:Int = it.value.toString().toInt()
                Toast.makeText(this,"Succesful Reading Data", Toast.LENGTH_SHORT).show()
                binding.statusDaun.setText(label.toString())
            }else{
                Toast.makeText(this,"Reading Error",Toast.LENGTH_SHORT).show()
            }
        }
        database_ref2 = FirebaseDatabase.getInstance().getReference("/")
        database_ref2.child("label_ph").get().addOnSuccessListener {
            if(it.exists()){
                val label:Int = it.value.toString().toInt()
                Toast.makeText(this,"Succesful Reading Data", Toast.LENGTH_SHORT).show()
                binding.phMeter.setText(label.toString())
            }else{
                Toast.makeText(this,"Reading Error",Toast.LENGTH_SHORT).show()
            }
        }
        database_ref3 = FirebaseDatabase.getInstance().getReference("/")
        database_ref3.child("label_soil").get().addOnSuccessListener {
            if(it.exists()){
                val label:Int = it.value.toString().toInt()
                Toast.makeText(this,"Succesful Reading Data", Toast.LENGTH_SHORT).show()
                binding.soilMoisture.setText(label.toString())
            }else{
                Toast.makeText(this,"Reading Error",Toast.LENGTH_SHORT).show()
            }
        }
    }
    override fun onDestroy() {
        super.onDestroy()
        // Cancel the timer to prevent memory leaks
        timer.cancel()
    }
}