using Godot;
using System;

public partial class SecondMenu : Node2D
{
	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}
	public void _on_texture_button_pressed()
	{
		Global.SelectedTrack = 1;
		LoadScene("res://Scenes/ground.tscn");
	}
		public void _on_texture_button_2_pressed()
	{
		Global.SelectedTrack = 2;
		LoadScene("res://Scenes/ground.tscn");
	}
		public void _on_texture_button_3_pressed()
	{
		Global.SelectedTrack = 3;
		LoadScene("res://Scenes/ground.tscn");

	}
		public void _on_texture_button_4_pressed()
	{
		Global.SelectedTrack = 4;
		LoadScene("res://Scenes/ground.tscn");

	}
		public void _on_texture_button_5_pressed()
	{
		Global.SelectedTrack = 5;
		LoadScene("res://Scenes/ground.tscn");

	}
	public void _on_texture_button_6_pressed()
	{
		Global.SelectedTrack = 6;
		LoadScene("res://Scenes/ground.tscn");

	}
	public void _on_button_pressed()
	{
		LoadScene("res://Scenes/main_menu.tscn");
		
	}
	public void _on_parking_pressed()
	{
		Global.SelectedTrack = 7;
		LoadScene("res://Scenes/ground.tscn");
	}

	private void LoadScene(string scenePath)
{
	PackedScene scene = GD.Load<PackedScene>(scenePath);
	Node instance = scene.Instantiate();
	GetTree().Root.AddChild(instance);

	// Optional: Remove the current scene
	QueueFree(); // Frees the current scene node
}
	
}
