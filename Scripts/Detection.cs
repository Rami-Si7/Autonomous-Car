using Godot;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;

public class Midpoint
{
    [JsonProperty("x")]
    public float X { get; set; }

    [JsonProperty("y")]
    public float Y { get; set; }

    public static implicit operator Midpoint(Vector2 v)
    {
        throw new NotImplementedException();
    }
}

// Detection class for bounding boxes and classes
public class Detection
{
    [JsonProperty("bbox")]
    public List<float> Bbox { get; set; }

    [JsonProperty("class")]
    public string Class { get; set; }

    [JsonProperty("confidence")]
    public float Confidence { get; set; }
}

// Detection response including detections and midpoint
public class DetectionResponse
{
    [JsonProperty("detections")]
    public List<Detection> Detections { get; set; }

    [JsonProperty("midpoint")]
    public Midpoint Midpoint { get; set; } // Midpoint property
}