package tileworld.agent;

public class Message {
	public enum MessageType {
		GENERIC,
		INFO,
		INTENTION
	}

	private String from; // the sender
	private String to; // the recepient
	private String message; // the message
	private MessageType type;
	private String entityType;
	private int x;
	private int y;
	private int senderX;
	private int senderY;
	
	public Message(String from, String to, String message){
		this(from, to, message, MessageType.GENERIC, "", -1, -1, -1, -1);
	}

	public Message(String from, String to, String message, MessageType type, String entityType, int x, int y, int senderX, int senderY){
		this.from = from;
		this.to = to;
		this.message = message;
		this.type = type;
		this.entityType = entityType;
		this.x = x;
		this.y = y;
		this.senderX = senderX;
		this.senderY = senderY;
	}

	public static Message info(String from, String entityType, int x, int y, int senderX, int senderY) {
		return new Message(from, "*", "info", MessageType.INFO, entityType, x, y, senderX, senderY);
	}

	public static Message intention(String from, String entityType, int x, int y, int senderX, int senderY) {
		return new Message(from, "*", "intent", MessageType.INTENTION, entityType, x, y, senderX, senderY);
	}

	public String getFrom() {
		return from;
	}

	public String getTo() {
		return to;
	}

	public String getMessage() {
		return message;
	}

	public MessageType getType() {
		return type;
	}

	public String getEntityType() {
		return entityType;
	}

	public int getX() {
		return x;
	}

	public int getY() {
		return y;
	}

	public int getSenderX() {
		return senderX;
	}

	public int getSenderY() {
		return senderY;
	}

}
