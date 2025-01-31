using Godot;
using System;

public partial class MainMenu : Node2D
{
	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}
	public void _on_autonomous_pressed()
	{
		Global.GameMode = "Autonomous";
		LoadScene("res://Scenes/second_menu.tscn");

	}
	public void _on_training_pressed()
	{
		Global.GameMode = "Training";
		LoadScene("res://Scenes/second_menu.tscn");
	}
	private void LoadScene(string scenePath)
	{
		PackedScene scene = GD.Load<PackedScene>(scenePath);
		Node instance = scene.Instantiate();
		GetTree().Root.AddChild(instance);

		QueueFree();
	}
}
