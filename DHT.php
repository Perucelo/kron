<?php
$servername = "bzayjfy8zw7gkcajc7aj-mysql.services.clever-cloud.com";
$database = "bzayjfy8zw7gkcajc7aj";
$username = "u4gnbqjncr8kcznp";
$password = "clFpGRTokoJfirSUAj9f";
// Create connection
$conn = mysqli_connect($servername, $username, $password, $database);
// Check connection
if (!$conn) {
    die("Connection failed: " . mysqli_connect_error());
}
echo "Connected successfully";
mysqli_close($conn);
?>